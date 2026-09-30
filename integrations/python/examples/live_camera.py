"""Display the Scout camera until Q or Escape is pressed."""

import cv2

from moorebot_scout import ScoutClient


def main() -> None:
    with ScoutClient() as scout:
        print("Camera connected. Press Q or Escape in the image window to stop.")
        while True:
            frame = scout.read_image()
            cv2.imshow("Moorebot Scout", frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
