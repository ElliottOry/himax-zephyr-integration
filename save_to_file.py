import serial

COM_PORT = "/dev/tty.usbserial-0289815B"          # ← set this to your port
BAUD_RATE = 921600         # ← set this to your device
IMAGE_WIDTH = 160
IMAGE_HEIGHT = 120
# Total number of bytes that make up a single 8-bit grayscale frame
FRAME_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT  # 19 200 bytes


def _normalize_frame(frame: object) -> bytes:
    if isinstance(frame, (bytes, bytearray, memoryview)):
        raw = bytes(frame)
        if len(raw) == FRAME_SIZE:
            data = raw

        elif len(raw) == FRAME_SIZE // 8:
            unpacked = bytearray(FRAME_SIZE)

            out_idx = 0
            for byte in raw:
                for shift in range(7, -1, -1):
                    bit = (byte >> shift) & 0x01

                    unpacked[out_idx] = 0xFF if bit else 0x00
                    out_idx += 1

            data = bytes(unpacked)

        else:
            data = raw
    elif hasattr(frame, '__iter__') and not isinstance(frame, str):
        try:
            data = bytes(frame)
        except (TypeError, ValueError):
            data = None
        if data is None:
            raise TypeError(
                "Iterable *frame* must yield integers in the range 0..255"
            )

    elif isinstance(frame, (str, bytes, bytearray)):

        bit_str = frame.decode() if isinstance(frame, (bytes, bytearray)) else frame
        if any(ch not in '01' for ch in bit_str):
            raise ValueError("Binary string *frame* may only contain '0' and '1' characters")

        if len(bit_str) != FRAME_SIZE:
            raise ValueError(
                f"Expected {FRAME_SIZE} bits for a {IMAGE_WIDTH}x{IMAGE_HEIGHT} image, "
                f"got {len(bit_str)}"
            )
        
        data = bytes(0xFF if ch == '1' else 0x00 for ch in bit_str)

    else:
        raise TypeError(
            "Unsupported *frame* type – expected bytes-like, iterable of ints, "
            "or a binary 0/1 string"
        )

    if len(data) != FRAME_SIZE:
        raise ValueError(
            f"Expected {FRAME_SIZE} bytes for a {IMAGE_WIDTH}x{IMAGE_HEIGHT} "
            f"image, got {len(data)}"
        )

    return data


def save_frame_to_file(frame: object, filename: str) -> None:

    data = _normalize_frame(frame)

    with open(filename, "wb") as fp:
        fp.write(data)

save_to_file = save_frame_to_file 


def _write_frame_to_disk(frame: bytes, idx: int) -> str:
    outfn = f"image_{idx:03d}.bin"
    with open(outfn, "wb") as fp:
        fp.write(frame)
    return outfn


def grab_frames_from_serial(port: str, baud: int):

    ser = serial.Serial(port, baud, timeout=1)
    print(f"Listening on {port} at {baud} baud - waiting for <FRAME> markers …")

    frame_counter = 0
    marker_start = b"<FRAME>"
    marker_end = b"</FRAME>"


    window = bytearray(max(len(marker_start), len(marker_end)))

    while True:
        b = ser.read(1)
        if not b:
            continue

        window.append(b[0])
        if len(window) > len(marker_start):
            window.pop(0)

        if window.endswith(marker_start):

            next_byte = ser.read(1)
            if next_byte != b"\n":
                frame_first_byte = next_byte
            else:
                frame_first_byte = b""

            to_read = FRAME_SIZE - len(frame_first_byte)
            frame_rest = ser.read(to_read)
            frame = frame_first_byte + frame_rest
            if len(frame) != FRAME_SIZE:
                print("Incomplete frame - discarding")
                continue  # restart marker search

            trailer = ser.read(len(marker_end))

            if trailer == marker_end:
                ser.read(1)  # consume optional '\n' (non-blocking for timeout)

            if trailer != marker_end:
                print("Missing </FRAME> marker - discarding")
                continue

            # All good – write the frame to disk.
            frame_counter += 1
            filename = _write_frame_to_disk(frame, frame_counter)
            print(f"Saved frame {frame_counter} - {filename} ({len(frame)} bytes)")
            
if __name__ == "__main__":
    grab_frames_from_serial(COM_PORT, BAUD_RATE)
