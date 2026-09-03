use moorebot_scout::{services::KNOWN_SERVICES, topics::KNOWN_TOPICS};

fn main() {
    println!("Known Scout topics:");
    for topic in KNOWN_TOPICS {
        println!(
            "  {:<42} {:<28} {:?}",
            topic.name, topic.capability, topic.support
        );
    }

    println!("\nKnown Scout services (discovery-only):");
    for service in KNOWN_SERVICES {
        println!("  {:<42} {}", service.name, service.capability);
    }
}
