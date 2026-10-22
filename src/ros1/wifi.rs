//! One-shot service transport with bounded allocation and a shared deadline.
//!
//! rosrust's service client does not enforce the topic-message limits on its
//! service headers and replies. Keep this credential-changing operation on its
//! own connection, without creating a node that must unregister after Wi-Fi drops.

use super::{Ros1Config, SCOUT_INTERNAL_HOSTNAME};
use crate::{
    services::WIFI_SWITCH_SSID_KEY,
    wifi::{WifiCredentials, WifiProvisioningError as Error, WifiProvisioningResult as Outcome},
};
use std::{
    io::{self, Read, Write},
    net::{IpAddr, SocketAddr, TcpStream, ToSocketAddrs},
    sync::mpsc,
    time::{Duration, Instant},
};

const SERVICE_TYPE: &str = "roller_eye/wifi_switch_ssid_key";
const SERVICE_MD5: &str = "879dd501d4c8e7d6703c458f6fc7737c";
const MAX_HTTP_HEADER: usize = 16 * 1024;
const MAX_LOOKUP_BODY: usize = 64 * 1024;
const MAX_SERVICE_HEADER: usize = 64 * 1024;
const MAX_SERVICE_ERROR: usize = 1024;

/// Ask the Scout to join a WPA/WPA2 personal router, using a dedicated ROS
/// service connection. Calling [`super::init`] first is unnecessary.
///
/// The overall timeout (1–60 seconds) covers discovery, connection, handshake,
/// sending, and acknowledgement. Allocations are bounded before reading peer
/// bodies. The request is sent once and is never retried automatically.
///
/// `Accepted` means the firmware queued the change, not that the router accepted
/// its credentials. Losing the reply after sending begins returns
/// `DeliveryUncertain`; the robot may already have changed networks.
///
/// Native ROS 1 sends credentials without encryption. Connect from a trusted
/// network, normally the Scout's access point. The published firmware may log
/// credentials on the robot; this client never includes them in diagnostics.
pub fn provision_wifi(
    config: &Ros1Config,
    credentials: &WifiCredentials,
    timeout: Duration,
) -> Result<Outcome, Error> {
    if !(Duration::from_secs(1)..=Duration::from_secs(60)).contains(&timeout) {
        return Err(Error::InvalidConfiguration(
            "Wi-Fi setup timeout must be 1–60 seconds",
        ));
    }
    let deadline = Instant::now() + timeout;
    let master = Endpoint::parse(&config.master_uri, "http://", true)?;
    let caller = caller_id(&config.node_name)?;
    let (service_uri, master_address) = lookup_service(&master, &caller, deadline)?;
    let service = Endpoint::parse(&service_uri, "rosrpc://", false)?;
    let addresses = if service.host == SCOUT_INTERNAL_HOSTNAME {
        vec![SocketAddr::new(master_address.ip(), service.port)]
    } else {
        service.resolve(deadline)?
    };
    let mut connection = connect(&addresses, deadline)?;
    handshake(&mut connection, &caller)?;
    send_credentials(&mut connection, credentials)
}

fn caller_id(name: &str) -> Result<String, Error> {
    if name.is_empty()
        || name.len() > 256
        || !name
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'/' | b'_'))
    {
        return Err(Error::InvalidConfiguration(
            "Wi-Fi setup node name must use ASCII letters, digits, slash, or underscore",
        ));
    }
    Ok(if name.starts_with('/') {
        name.to_owned()
    } else {
        format!("/{name}")
    })
}

#[derive(Debug)]
struct Endpoint {
    host: String,
    port: u16,
    authority: String,
    path: String,
}

impl Endpoint {
    fn parse(uri: &str, scheme: &str, allow_path: bool) -> Result<Self, Error> {
        let invalid = || Error::InvalidConfiguration("invalid ROS master or service URI");
        if uri.len() > 2048 || uri.bytes().any(|b| b <= b' ' || b >= 127) {
            return Err(invalid());
        }
        let rest = uri.strip_prefix(scheme).ok_or_else(invalid)?;
        let (authority, path) = rest
            .split_once('/')
            .map_or((rest, "/".to_owned()), |(a, p)| (a, format!("/{p}")));
        if authority.contains(['@', '?', '#'])
            || path.contains(['?', '#'])
            || (!allow_path && path != "/")
        {
            return Err(invalid());
        }
        let (host, port) = if let Some(bracketed) = authority.strip_prefix('[') {
            let (host, tail) = bracketed.split_once(']').ok_or_else(invalid)?;
            if host.parse::<std::net::Ipv6Addr>().is_err() {
                return Err(invalid());
            }
            (host, tail.strip_prefix(':').ok_or_else(invalid)?)
        } else {
            let (host, port) = authority.rsplit_once(':').ok_or_else(invalid)?;
            if host.is_empty()
                || !host
                    .bytes()
                    .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'.' | b'-' | b'_'))
            {
                return Err(invalid());
            }
            (host, port)
        };
        let port = port.parse::<u16>().map_err(|_| invalid())?;
        if port == 0 {
            return Err(invalid());
        }
        Ok(Self {
            host: host.into(),
            port,
            authority: authority.into(),
            path,
        })
    }

    fn resolve(&self, deadline: Instant) -> Result<Vec<SocketAddr>, Error> {
        if let Ok(ip) = self.host.parse::<IpAddr>() {
            return Ok(vec![SocketAddr::new(ip, self.port)]);
        }
        // System DNS can block indefinitely. Only the resolver runs in this
        // worker; a timed-out worker never receives credentials or opens sockets.
        let (sender, receiver) = mpsc::sync_channel(1);
        let host = self.host.clone();
        let port = self.port;
        std::thread::Builder::new()
            .name("scout-wifi-dns".into())
            .spawn(move || {
                let addresses = (host.as_str(), port)
                    .to_socket_addrs()
                    .map(|addresses| addresses.take(8).collect::<Vec<_>>())
                    .map_err(|error| error.kind());
                let _ = sender.send(addresses);
            })
            .map_err(transport_error)?;
        match receiver.recv_timeout(remaining(deadline)?) {
            Ok(Ok(mut addresses)) if !addresses.is_empty() => {
                addresses.sort_by_key(|address| !address.is_ipv4());
                Ok(addresses)
            }
            Ok(Ok(_)) => Err(Error::Transport(io::ErrorKind::NotFound)),
            Ok(Err(kind)) => Err(Error::Transport(kind)),
            Err(mpsc::RecvTimeoutError::Timeout) => Err(Error::Timeout),
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                Err(Error::Transport(io::ErrorKind::Other))
            }
        }
    }
}

fn remaining(deadline: Instant) -> Result<Duration, Error> {
    deadline
        .checked_duration_since(Instant::now())
        .filter(|left| !left.is_zero())
        .ok_or(Error::Timeout)
}

fn transport_error(error: io::Error) -> Error {
    match error.kind() {
        io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock => Error::Timeout,
        kind => Error::Transport(kind),
    }
}

struct Connection {
    stream: TcpStream,
    deadline: Instant,
}

impl Read for Connection {
    fn read(&mut self, output: &mut [u8]) -> io::Result<usize> {
        let left =
            remaining(self.deadline).map_err(|_| io::Error::from(io::ErrorKind::TimedOut))?;
        self.stream.set_read_timeout(Some(left))?;
        self.stream.read(output)
    }
}

impl Write for Connection {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        let left =
            remaining(self.deadline).map_err(|_| io::Error::from(io::ErrorKind::TimedOut))?;
        self.stream.set_write_timeout(Some(left))?;
        self.stream.write(bytes)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.stream.flush()
    }
}

fn connect(addresses: &[SocketAddr], deadline: Instant) -> Result<Connection, Error> {
    let mut last = Error::Transport(io::ErrorKind::NotFound);
    for address in addresses.iter().take(8) {
        match TcpStream::connect_timeout(address, remaining(deadline)?.min(Duration::from_secs(2)))
        {
            Ok(stream) => return Ok(Connection { stream, deadline }),
            Err(error) => last = transport_error(error),
        }
    }
    Err(last)
}

fn lookup_service(
    master: &Endpoint,
    caller: &str,
    deadline: Instant,
) -> Result<(String, SocketAddr), Error> {
    let mut connection = connect(&master.resolve(deadline)?, deadline)?;
    let master_address = connection.stream.peer_addr().map_err(transport_error)?;
    // caller_id permits only characters safe in XML text.
    let body = format!(
        "<?xml version=\"1.0\"?><methodCall><methodName>lookupService</methodName><params><param><value><string>{caller}</string></value></param><param><value><string>{WIFI_SWITCH_SSID_KEY}</string></value></param></params></methodCall>"
    );
    let request = format!(
        "POST {} HTTP/1.0\r\nHost: {}\r\nContent-Type: text/xml\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
        master.path,
        master.authority,
        body.len(),
        body
    );
    connection
        .write_all(request.as_bytes())
        .map_err(transport_error)?;
    let bytes = read_http_body(&mut connection)?;
    let text = std::str::from_utf8(&bytes)
        .map_err(|_| Error::Protocol("invalid ROS lookup response encoding"))?;
    let document =
        roxmltree::Document::parse(text).map_err(|_| Error::Protocol("invalid ROS lookup XML"))?;
    let root = document.root_element();
    if root.tag_name().name() != "methodResponse" {
        return Err(Error::Protocol("unexpected ROS lookup response"));
    }
    let mut current = root;
    for name in ["params", "param", "value", "array", "data"] {
        current = only_child(current, name)?;
    }
    let values = current
        .children()
        .filter(|n| n.is_element())
        .collect::<Vec<_>>();
    if values.len() != 3 || values.iter().any(|n| n.tag_name().name() != "value") {
        return Err(Error::Protocol("invalid ROS lookup result"));
    }
    let code = values[0]
        .children()
        .filter(|n| n.is_element())
        .collect::<Vec<_>>();
    if code.len() != 1 || !matches!(code[0].tag_name().name(), "int" | "i4") {
        return Err(Error::Protocol("invalid ROS lookup status"));
    }
    if code[0]
        .text()
        .and_then(|text| text.trim().parse::<i32>().ok())
        != Some(1)
    {
        return Err(Error::ServiceUnavailable);
    }
    let uri = only_child(values[2], "string")?
        .text()
        .ok_or(Error::Protocol("missing ROS service URI"))?;
    Ok((uri.to_owned(), master_address))
}

fn only_child<'a, 'input>(
    node: roxmltree::Node<'a, 'input>,
    name: &str,
) -> Result<roxmltree::Node<'a, 'input>, Error> {
    let mut children = node.children().filter(|n| n.is_element());
    match (children.next(), children.next()) {
        (Some(child), None) if child.tag_name().name() == name => Ok(child),
        _ => Err(Error::Protocol("unexpected ROS lookup XML structure")),
    }
}

fn read_http_body(connection: &mut Connection) -> Result<Vec<u8>, Error> {
    let mut header = Vec::new();
    while !header.ends_with(b"\r\n\r\n") {
        if header.len() >= MAX_HTTP_HEADER {
            return Err(Error::Protocol("ROS lookup HTTP header exceeds limit"));
        }
        let mut byte = [0];
        connection.read_exact(&mut byte).map_err(transport_error)?;
        header.push(byte[0]);
    }
    let header = std::str::from_utf8(&header)
        .map_err(|_| Error::Protocol("invalid ROS lookup HTTP header"))?;
    let mut lines = header.split("\r\n");
    let mut status = lines.next().unwrap_or_default().split_whitespace();
    if !matches!(status.next(), Some("HTTP/1.0" | "HTTP/1.1")) || status.next() != Some("200") {
        return Err(Error::Protocol("ROS lookup returned an HTTP error"));
    }
    let mut length = None;
    for line in lines.filter(|line| !line.is_empty()) {
        let (name, value) = line
            .split_once(':')
            .ok_or(Error::Protocol("invalid ROS lookup HTTP field"))?;
        if name.eq_ignore_ascii_case("transfer-encoding") {
            return Err(Error::Protocol("unsupported ROS lookup transfer encoding"));
        }
        if name.eq_ignore_ascii_case("content-length") {
            if length.is_some() {
                return Err(Error::Protocol("duplicate ROS lookup content length"));
            }
            length = Some(
                value
                    .trim()
                    .parse::<usize>()
                    .map_err(|_| Error::Protocol("invalid ROS lookup content length"))?,
            );
        }
    }
    let length = length.ok_or(Error::Protocol("missing ROS lookup content length"))?;
    if length > MAX_LOOKUP_BODY {
        return Err(Error::Protocol("ROS lookup body exceeds limit"));
    }
    let mut body = vec![0; length];
    connection.read_exact(&mut body).map_err(transport_error)?;
    Ok(body)
}

fn read_frame(reader: &mut impl Read, limit: usize) -> Result<Vec<u8>, Error> {
    let mut length = [0; 4];
    reader.read_exact(&mut length).map_err(transport_error)?;
    let length = u32::from_le_bytes(length) as usize;
    if length > limit {
        return Err(Error::Protocol("ROS service frame exceeds limit"));
    }
    let mut body = vec![0; length];
    reader.read_exact(&mut body).map_err(transport_error)?;
    Ok(body)
}

fn write_frame(writer: &mut impl Write, body: &[u8]) -> io::Result<()> {
    writer.write_all(&(body.len() as u32).to_le_bytes())?;
    writer.write_all(body)
}

fn handshake(connection: &mut Connection, caller: &str) -> Result<(), Error> {
    let mut header = Vec::new();
    for field in [
        format!("callerid={caller}"),
        format!("service={WIFI_SWITCH_SSID_KEY}"),
        format!("md5sum={SERVICE_MD5}"),
        format!("type={SERVICE_TYPE}"),
        "persistent=0".into(),
    ] {
        write_frame(&mut header, field.as_bytes()).map_err(transport_error)?;
    }
    write_frame(connection, &header).map_err(transport_error)?;
    let body = read_frame(connection, MAX_SERVICE_HEADER)?;
    let mut cursor = body.as_slice();
    let mut fields = std::collections::HashMap::new();
    while !cursor.is_empty() {
        if fields.len() >= 128 {
            return Err(Error::Protocol("too many ROS service header fields"));
        }
        let field = read_frame(&mut cursor, MAX_SERVICE_HEADER)?;
        let field = std::str::from_utf8(&field)
            .map_err(|_| Error::Protocol("invalid ROS service header encoding"))?;
        let (name, value) = field
            .split_once('=')
            .ok_or(Error::Protocol("invalid ROS service header field"))?;
        if fields.insert(name.to_owned(), value.to_owned()).is_some() {
            return Err(Error::Protocol("duplicate ROS service header field"));
        }
    }
    if fields.contains_key("error") {
        return Err(Error::ServiceFailed);
    }
    if fields.get("md5sum").map(String::as_str) != Some(SERVICE_MD5) {
        return Err(Error::Protocol(
            "Scout Wi-Fi service schema does not match the supported firmware",
        ));
    }
    if fields
        .get("type")
        .is_some_and(|service_type| service_type != SERVICE_TYPE)
    {
        return Err(Error::Protocol(
            "Scout Wi-Fi service type does not match the supported firmware",
        ));
    }
    Ok(())
}

fn send_credentials(
    connection: &mut Connection,
    credentials: &WifiCredentials,
) -> Result<Outcome, Error> {
    // A partial write can coincide with the network switch. From this point on,
    // never report a connection failure as a definite rejection or retry it.
    if write_frame(connection, &credentials.encode_request()).is_err() {
        return Ok(Outcome::DeliveryUncertain);
    }
    let mut success = [0];
    if connection.read_exact(&mut success).is_err() {
        return Ok(Outcome::DeliveryUncertain);
    }
    match success[0] {
        1 => match read_frame(connection, 1) {
            Ok(body) if body.as_slice() == [0] => Ok(Outcome::Accepted),
            Ok(body) if body.len() == 1 => Err(Error::Rejected(body[0] as i8)),
            _ => Ok(Outcome::DeliveryUncertain),
        },
        0 => match read_frame(connection, MAX_SERVICE_ERROR) {
            Ok(_) => Err(Error::ServiceFailed),
            Err(_) => Ok(Outcome::DeliveryUncertain),
        },
        _ => Ok(Outcome::DeliveryUncertain),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{net::TcpListener, thread};

    enum Reply {
        Status(i8),
        Disconnect,
        Truncated,
        Oversized,
        Failure,
        WrongMd5,
        OversizedHeader,
        Delay,
    }

    fn mock_setup(reply: Reply) -> (Result<Outcome, Error>, Vec<u8>, Duration) {
        let master = TcpListener::bind("127.0.0.1:0").unwrap();
        let service = TcpListener::bind("127.0.0.1:0").unwrap();
        let service_port = service.local_addr().unwrap().port();
        let config = Ros1Config {
            master_uri: format!("http://{}", master.local_addr().unwrap()),
            ..Ros1Config::default()
        };
        let master_thread = thread::spawn(move || {
            let (mut socket, _) = master.accept().unwrap();
            socket
                .set_read_timeout(Some(Duration::from_secs(2)))
                .unwrap();
            let mut connection = Connection {
                stream: socket.try_clone().unwrap(),
                deadline: Instant::now() + Duration::from_secs(2),
            };
            // Read the discovery request, checking it contains no credentials.
            let mut request = Vec::new();
            while !request.ends_with(b"\r\n\r\n") {
                let mut byte = [0];
                connection.read_exact(&mut byte).unwrap();
                request.push(byte[0]);
                assert!(request.len() <= MAX_HTTP_HEADER);
            }
            let headers = String::from_utf8(request).unwrap();
            assert!(headers.starts_with("POST / HTTP/1.0\r\n"));
            let length = headers
                .lines()
                .find_map(|line| line.strip_prefix("Content-Length: "))
                .unwrap()
                .parse::<usize>()
                .unwrap();
            let mut body = vec![0; length];
            connection.read_exact(&mut body).unwrap();
            let body = String::from_utf8(body).unwrap();
            assert!(body.contains("<methodName>lookupService</methodName>"));
            assert!(body.contains(WIFI_SWITCH_SSID_KEY));
            assert!(!body.contains("password"));
            assert!(!body.contains("private-ssid"));
            let response = format!(
                "<?xml version=\"1.0\"?><methodResponse><params><param><value><array><data><value><int>1</int></value><value><string>ok</string></value><value><string>rosrpc://linaro-alip:{service_port}</string></value></data></array></value></param></params></methodResponse>"
            );
            write!(
                socket,
                "HTTP/1.0 200 OK\r\nContent-Length: {}\r\n\r\n{}",
                response.len(),
                response
            )
            .unwrap();
        });
        let service_thread = thread::spawn(move || {
            let (mut socket, _) = service.accept().unwrap();
            socket
                .set_read_timeout(Some(Duration::from_secs(2)))
                .unwrap();
            let header = read_frame(&mut socket, MAX_SERVICE_HEADER).unwrap();
            assert!(
                header
                    .windows(SERVICE_MD5.len())
                    .any(|bytes| bytes == SERVICE_MD5.as_bytes())
            );
            for secret in [b"password".as_slice(), b"private-ssid".as_slice()] {
                assert!(!header.windows(secret.len()).any(|bytes| bytes == secret));
            }
            if matches!(reply, Reply::OversizedHeader) {
                socket.write_all(&u32::MAX.to_le_bytes()).unwrap();
            } else {
                let md5 = if matches!(reply, Reply::WrongMd5) {
                    "incompatible-schema"
                } else {
                    SERVICE_MD5
                };
                let mut fields = Vec::new();
                for field in [
                    format!("md5sum={md5}"),
                    format!("type={SERVICE_TYPE}"),
                    "callerid=/WiFiNode".into(),
                ] {
                    write_frame(&mut fields, field.as_bytes()).unwrap();
                }
                write_frame(&mut socket, &fields).unwrap();
            }
            if matches!(reply, Reply::WrongMd5 | Reply::OversizedHeader) {
                let mut probe = [0];
                assert_eq!(
                    socket.read(&mut probe).unwrap(),
                    0,
                    "credentials sent before a valid handshake"
                );
                return Vec::new();
            }
            let request = read_frame(&mut socket, 128).unwrap();
            assert_eq!(request, b"\x01\x0c\0\0\0private-ssid\x08\0\0\0password");
            match reply {
                Reply::Status(status) => {
                    socket.write_all(&[1, 1, 0, 0, 0, status as u8]).unwrap();
                }
                Reply::Disconnect => {}
                Reply::Truncated => {
                    socket.write_all(&[1, 1, 0, 0, 0]).unwrap();
                }
                Reply::Oversized => {
                    socket.write_all(&[1, 255, 255, 255, 255]).unwrap();
                }
                Reply::Failure => {
                    socket.write_all(&[0]).unwrap();
                    write_frame(&mut socket, b"password private-ssid secret-password").unwrap();
                }
                Reply::Delay => thread::sleep(Duration::from_millis(1200)),
                Reply::WrongMd5 | Reply::OversizedHeader => unreachable!(),
            }
            request
        });
        let credentials = WifiCredentials::new("private-ssid", "password").unwrap();
        let started = Instant::now();
        let result = provision_wifi(&config, &credentials, Duration::from_secs(1));
        let elapsed = started.elapsed();
        master_thread.join().unwrap();
        let request = service_thread.join().unwrap();
        (result, request, elapsed)
    }

    #[test]
    fn discovers_service_and_sends_station_credentials_once() {
        let (result, request, _) = mock_setup(Reply::Status(0));
        assert_eq!(result, Ok(Outcome::Accepted));
        assert!(!request.is_empty());
    }

    #[test]
    fn rejects_nonzero_firmware_status() {
        assert_eq!(mock_setup(Reply::Status(-1)).0, Err(Error::Rejected(-1)));
    }

    #[test]
    fn lost_or_invalid_reply_is_uncertain_not_success() {
        for reply in [Reply::Disconnect, Reply::Truncated, Reply::Oversized] {
            assert_eq!(mock_setup(reply).0, Ok(Outcome::DeliveryUncertain));
        }
    }

    #[test]
    fn server_failure_text_is_redacted() {
        let error = mock_setup(Reply::Failure).0.unwrap_err();
        assert_eq!(error, Error::ServiceFailed);
        let formatted = format!("{error}: {error:?}");
        for secret in ["password", "private-ssid", "secret-password"] {
            assert!(!formatted.contains(secret));
        }
    }

    #[test]
    fn refuses_incompatible_or_oversized_handshake_before_credentials() {
        for reply in [Reply::WrongMd5, Reply::OversizedHeader] {
            let (result, request, _) = mock_setup(reply);
            assert!(matches!(result, Err(Error::Protocol(_))));
            assert!(request.is_empty());
        }
    }

    #[test]
    fn missing_acknowledgement_respects_overall_timeout() {
        let (result, _, elapsed) = mock_setup(Reply::Delay);
        assert_eq!(result, Ok(Outcome::DeliveryUncertain));
        assert!(
            elapsed < Duration::from_millis(1150),
            "request exceeded its deadline: {elapsed:?}"
        );
    }

    #[test]
    fn announced_lengths_are_checked_before_reading_bodies() {
        assert!(matches!(
            read_frame(&mut u32::MAX.to_le_bytes().as_slice(), 1024),
            Err(Error::Protocol(_))
        ));
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let destination = listener.local_addr().unwrap();
        let server = thread::spawn(move || {
            let (mut socket, _) = listener.accept().unwrap();
            socket
                .write_all(b"HTTP/1.0 200 OK\r\nContent-Length: 4294967295\r\n\r\n")
                .unwrap();
        });
        let mut connection =
            connect(&[destination], Instant::now() + Duration::from_secs(1)).unwrap();
        assert!(matches!(
            read_http_body(&mut connection),
            Err(Error::Protocol(_))
        ));
        server.join().unwrap();
    }

    #[test]
    fn slow_trickle_does_not_extend_deadline() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let destination = listener.local_addr().unwrap();
        let server = thread::spawn(move || {
            let (mut socket, _) = listener.accept().unwrap();
            for byte in b"HTTP/1.0 200 OK\r\nContent-Length: 1\r\n\r\nx" {
                if socket.write_all(&[*byte]).is_err() {
                    break;
                }
                thread::sleep(Duration::from_millis(20));
            }
        });
        let started = Instant::now();
        let mut connection = connect(&[destination], started + Duration::from_millis(100)).unwrap();
        assert_eq!(read_http_body(&mut connection), Err(Error::Timeout));
        assert!(started.elapsed() < Duration::from_millis(300));
        drop(connection);
        server.join().unwrap();
    }

    #[test]
    fn validates_endpoint_and_timeout_before_contacting_network() {
        assert_eq!(
            Endpoint::parse("rosrpc://[::1]:1234", "rosrpc://", false)
                .unwrap()
                .host,
            "::1"
        );
        for uri in [
            "rosrpc://robot",
            "rosrpc://robot:0",
            "rosrpc://user@robot:123",
            "rosrpc://robot:123/path",
            "rosrpc://robot:123\r\nInjected: true",
        ] {
            assert!(Endpoint::parse(uri, "rosrpc://", false).is_err());
        }
        let credentials = WifiCredentials::new("router", "password").unwrap();
        for timeout in [Duration::ZERO, Duration::from_secs(61)] {
            assert!(matches!(
                provision_wifi(&Ros1Config::default(), &credentials, timeout),
                Err(Error::InvalidConfiguration(_))
            ));
        }
    }
}
