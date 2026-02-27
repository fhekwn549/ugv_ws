"""OccupancyGrid → PNG conversion with caching."""

import io
import struct
import zlib


def _create_png(width: int, height: int, pixels: bytes) -> bytes:
    """Create a minimal PNG from raw RGBA pixel bytes.

    Uses stdlib only (no Pillow dependency).
    """

    def _chunk(chunk_type: bytes, data: bytes) -> bytes:
        c = chunk_type + data
        crc = struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)
        return struct.pack(">I", len(data)) + c + crc

    # IHDR
    ihdr_data = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    ihdr = _chunk(b"IHDR", ihdr_data)

    # IDAT — raw pixel rows with filter byte 0 (None)
    raw_rows = bytearray()
    stride = width * 4
    for y in range(height):
        raw_rows.append(0)  # filter type None
        raw_rows.extend(pixels[y * stride:(y + 1) * stride])
    compressed = zlib.compress(bytes(raw_rows), 6)
    idat = _chunk(b"IDAT", compressed)

    # IEND
    iend = _chunk(b"IEND", b"")

    signature = b"\x89PNG\r\n\x1a\n"
    return signature + ihdr + idat + iend


def occupancy_grid_to_png(width: int, height: int,
                          data: list[int]) -> bytes:
    """Convert OccupancyGrid data[] to a PNG image (bytes).

    Colors:
        -1 (unknown)  → dark grey  (40, 43, 55)
         0 (free)     → light grey (230, 233, 240)
         1-100 (occupied) → black gradient
    The image is flipped vertically so row 0 is at the bottom (ROS convention).
    """
    pixels = bytearray(width * height * 4)

    for i, val in enumerate(data):
        if val == -1:
            r, g, b = 40, 43, 55
        elif val == 0:
            r, g, b = 230, 233, 240
        else:
            shade = max(0, 255 - int(val * 2.55))
            r, g, b = shade, shade, shade

        # Flip vertically: OccupancyGrid row 0 is bottom
        row = i // width
        col = i % width
        flipped_row = height - 1 - row
        idx = (flipped_row * width + col) * 4

        pixels[idx] = r
        pixels[idx + 1] = g
        pixels[idx + 2] = b
        pixels[idx + 3] = 255

    return _create_png(width, height, bytes(pixels))
