import argparse
import random

import requests
import cv2
import numpy as np
import time

def grab_image(limelight_ip):
    """
    Grab the latest snapshot from the Limelight camera at the given IP.

    Args:
        limelight_ip (str): The IP address of the Limelight camera.

    Returns:
        image (np.ndarray): The BGR image if successful, else None
        filename (str): The snapshot filename
    """
    try:
        # Generate URLs
        snapshot_url = f"http://{limelight_ip}/cam/snapshot"
        manifest_url = f"http://{limelight_ip}/cam/snapshot/manifest"

        # Request snapshot
        response = requests.get(snapshot_url)
        response.raise_for_status()
        time.sleep(0.2)  # Give the camera time to save the snapshot

        # Get snapshot manifest
        response = requests.get(manifest_url)
        response.raise_for_status()
        manifest = response.json()
        if not manifest:
            print("Manifest is empty")
            return None, None

        # Find latest snapshot
        latest_file = sorted(manifest, key=lambda x: x['timestamp'], reverse=True)[0]['filename']

        # Download latest snapshot
        file_url = f"http://{limelight_ip}/cam/snapshot/{latest_file}"
        response = requests.get(file_url)
        response.raise_for_status()

        # Convert to OpenCV image (BGR)
        image_array = np.frombuffer(response.content, dtype=np.uint8)
        image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        return image, latest_file

    except requests.RequestException as e:
        print("Error grabbing image:", e)
        return None, None


def main():
    # --- Parse CLI arguments ---
    parser = argparse.ArgumentParser(
        description="Live viewer for Limelight camera snapshots.",
        epilog="Press 'q' in the image window to exit."
    )
    parser.add_argument(
        "ip",
        type=str,
        help="IP address of the Limelight camera (e.g., 10.0.0.2)"
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=0.5,
        help="Snapshot update rate in frames per second (default: 0.5 fps, i.e., 2s per snapshot)"
    )
    args = parser.parse_args()

    limelight_ip = args.ip
    update_interval = 1.0 / args.fps if args.fps > 0 else 2.0

    last_filename = None

    try:
        while True:
            image, filename = grab_image(limelight_ip)

            if image is None or filename is None:
                # Skip this iteration if we failed to grab an image
                time.sleep(update_interval)
                continue

            # Skip if it’s the same snapshot as last time
            if filename == last_filename:
                time.sleep(update_interval)
                continue

            last_filename = filename
            print("Displaying snapshot:", filename)

            # Display the image in a window
            cv2.imshow("Limelight Live Snapshot", image)

            # Wait 1 ms to refresh window and check for quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Wait until next update
            time.sleep(update_interval)

    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
