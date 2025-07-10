import serial
import time
from PIL import Image
import numpy as np

COM_PORT = "/dev/tty.usbserial-0289815B"          # ← set this to your port
BAUD_RATE = 921600         # ← set this to your device
IMAGE_WIDTH = 160
IMAGE_HEIGHT = 120
FRAME_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT

def grab_frames_from_serial(port, baud):
    ser = serial.Serial(port, baud, timeout=1)
    print(f"Listening on {port} at {baud} baud...")
    images = []
    in_frame = False
    hex_lines = []
    while True:
        line = ser.readline()
        if not line:
            continue
        text = line.decode(errors='ignore').strip()
        if text == "<FRAME>":
            in_frame = True
            hex_lines = []
            print("Frame start detected")
        elif text == "</FRAME>":
            print("/FRAME>")
            if in_frame and hex_lines:
                hexdata = ''.join(hex_lines)
                if len(hexdata) == FRAME_SIZE * 2:
                    try:
                        img_bytes = bytes.fromhex(hexdata)
                        images.append(img_bytes)
                        idx = len(images)
                        # Save raw bytes
                        outfn = f"image_{idx:03d}.bin"
                        with open(outfn, "wb") as f:
                            f.write(img_bytes)
                        print(f"Image {idx}: {outfn} ({len(img_bytes)} bytes)")
                        # Decode and show
                        show_as_image(img_bytes, idx)
                    except ValueError:
                        print(f"Skipping image {len(images)+1}: invalid hex data")
                else:
                    print(f"Skipping image {len(images)+1}: wrong hex length ({len(hexdata)}, want {FRAME_SIZE*2})")
            in_frame = False
        elif in_frame and text and not text.startswith("<"):
            hex_lines.append(text)

def show_as_image(raw, idx):
    if len(raw) != FRAME_SIZE:
        
        print(f"Cannot display frame {idx}: expected {FRAME_SIZE} bytes, got {len(raw)}")
        return
    img = np.frombuffer(raw, dtype=np.uint8).reshape((IMAGE_HEIGHT, IMAGE_WIDTH))
    im = Image.fromarray(img, mode='L')
    im.show(title=f"Frame {idx}")

if __name__ == "__main__":
    grab_frames_from_serial(COM_PORT, BAUD_RATE)
