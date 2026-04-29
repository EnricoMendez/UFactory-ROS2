
import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
from PIL import Image, ImageDraw
from skimage.measure import approximate_polygon, find_contours
from skimage.morphology import binary_closing, binary_opening, disk, remove_small_objects # type: ignore


DEFAULT_HEADER = """;Initialization
G90 ;Global reference
G21 ;Use mm as units
;Go home
G0 X200 Y0 Z200 A180 B0 C0
G4 P1 ;Wait 1 sec

;Go to initial position
G0 X200 Y0
G1 Z-9 F5000
G91 ;set incremental distance mode
"""

DEFAULT_FOOTER = """
;Ending
G0 Z20
G90 ;Global reference
G0 X200 Y0 Z200 A180 B0 C0
"""


@dataclass(frozen=True)
class TraceConfig:
    image_path: Path
    gcode_path: Path
    points_path: Path
    preview_path: Path
    threshold: int
    min_area_px: int
    simplify_tol_px: float
    canvas_width_mm: float
    canvas_height_mm: float
    start_x_mm: float
    start_y_mm: float
    travel_z_mm: float
    draw_z_mm: float
    feedrate: float
    close_loops: bool


def parse_args() -> TraceConfig:
    parser = argparse.ArgumentParser(
        description="Convierte una imagen a contornos y G-code incremental."
    )
    parser.add_argument("--image", default="airlab_logo.png", help="Imagen de entrada.")
    parser.add_argument(
        "--gcode-out", default="logo.gcode", help="Archivo de salida G-code."
    )
    parser.add_argument(
        "--points-out", default="logo_points.csv", help="Archivo CSV con puntos."
    )
    parser.add_argument(
        "--preview-out",
        default="logo_contours.png",
        help="Imagen con la visualizacion de contornos detectados.",
    )
    parser.add_argument(
        "--threshold",
        type=int,
        default=245,
        help="Pixeles por debajo de este valor gris se consideran dibujo.",
    )
    parser.add_argument(
        "--min-area-px",
        type=int,
        default=300, #80,
        help="Descarta regiones pequeñas en pixeles.",
    )
    parser.add_argument(
        "--simplify-tol-px",
        type=float,
        default=1.5, #2.5,
        help="Tolerancia de simplificacion de contornos en pixeles.",
    )
    parser.add_argument(
        "--canvas-width-mm",
        type=float,
        default=400.0,
        help="Ancho maximo del dibujo en mm.",
    )
    parser.add_argument(
        "--canvas-height-mm",
        type=float,
        default=180.0,
        help="Alto maximo del dibujo en mm.",
    )
    parser.add_argument(
        "--start-x-mm",
        type=float,
        default=0.0,
        help="Desplazamiento inicial en X dentro del area de dibujo.",
    )
    parser.add_argument(
        "--start-y-mm",
        type=float,
        default=0.0,
        help="Desplazamiento inicial en Y dentro del area de dibujo.",
    )
    parser.add_argument(
        "--travel-z-mm",
        type=float,
        default=20.0,
        help="Altura de pluma levantada.",
    )
    parser.add_argument(
        "--draw-z-mm",
        type=float,
        default=-20.0,
        help="Altura de dibujo respecto a la referencia usada por tu maquina.",
    )
    parser.add_argument(
        "--feedrate", type=float, default=5000.0, help="Feedrate para G1."
    )
    parser.add_argument(
        "--open-contours",
        action="store_true",
        help="No cierra automaticamente contornos.",
    )
    args = parser.parse_args()
    return TraceConfig(
        image_path=Path(args.image),
        gcode_path=Path(args.gcode_out),
        points_path=Path(args.points_out),
        preview_path=Path(args.preview_out),
        threshold=args.threshold,
        min_area_px=args.min_area_px,
        simplify_tol_px=args.simplify_tol_px,
        canvas_width_mm=args.canvas_width_mm,
        canvas_height_mm=args.canvas_height_mm,
        start_x_mm=args.start_x_mm,
        start_y_mm=args.start_y_mm,
        travel_z_mm=args.travel_z_mm,
        draw_z_mm=args.draw_z_mm,
        feedrate=args.feedrate,
        close_loops=not args.open_contours,
    )


def load_mask(image_path: Path, threshold: int, min_area_px: int) -> np.ndarray:
    image = Image.open(image_path).convert("L")
    pixels = np.asarray(image)
    mask = pixels < threshold
    mask = binary_closing(mask, disk(2))
    mask = binary_opening(mask, disk(1))
    mask = remove_small_objects(mask, min_size=min_area_px)
    return mask


def contour_area(points: np.ndarray) -> float:
    x = points[:, 0]
    y = points[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def extract_paths(mask: np.ndarray, simplify_tol_px: float) -> list[np.ndarray]:
    contours = find_contours(mask.astype(float), 0.5)
    paths: list[np.ndarray] = []
    for contour in contours:
        if len(contour) < 3:
            continue
        simplified = approximate_polygon(contour, tolerance=simplify_tol_px)
        if len(simplified) < 3:
            continue
        # find_contours devuelve (fila, columna). Mapeamos a X=fila, Y=columna.
        points = np.column_stack((simplified[:, 0], simplified[:, 1]))
        # Add first point at the end to close the loop 
        points = np.vstack((points, points[0:2]))

        if contour_area(points) < 10.0:
            continue
        paths.append(points)
        
    
    return sort_paths(paths)


def sort_paths(paths: Sequence[np.ndarray]) -> list[np.ndarray]:
    if not paths:
        return []
    remaining = [path.copy() for path in paths]
    ordered = [remaining.pop(0)]
    while remaining:
        last_point = ordered[-1][-1]
        best_idx = 0
        best_dist = math.inf
        best_path = remaining[0]
        for idx, path in enumerate(remaining):
            direct = np.linalg.norm(path[0] - last_point)
            reverse = np.linalg.norm(path[-1] - last_point)
            if direct < best_dist:
                best_idx = idx
                best_dist = direct
                best_path = path
            if reverse < best_dist:
                best_idx = idx
                best_dist = reverse
                best_path = path[::-1]
        ordered.append(best_path)
        remaining.pop(best_idx)
    return ordered


def scale_paths(
    paths: Sequence[np.ndarray],
    canvas_width_mm: float,
    canvas_height_mm: float,
    start_x_mm: float,
    start_y_mm: float,
) -> list[np.ndarray]:
    all_points = np.vstack(paths)
    min_xy = all_points.min(axis=0)
    max_xy = all_points.max(axis=0)
    size_xy = np.maximum(max_xy - min_xy, 1e-9)
    scale = min(canvas_width_mm / size_xy[0], canvas_height_mm / size_xy[1])

    scaled_paths = []
    for path in paths:
        shifted = path - min_xy
        scaled = shifted * scale
        scaled[:, 0] += start_x_mm
        scaled[:, 1] += start_y_mm
        scaled_paths.append(scaled)
    return scaled_paths


def write_points_csv(points_path: Path, paths: Sequence[np.ndarray]) -> None:
    with points_path.open("w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["path_id", "point_id", "x_mm", "y_mm"])
        for path_id, path in enumerate(paths):
            for point_id, (x_mm, y_mm) in enumerate(path):
                writer.writerow(
                    [path_id, point_id, f"{x_mm:.3f}", f"{y_mm:.3f}"]
                )


def path_color(path_id: int) -> tuple[int, int, int]:
    palette = [
        (255, 59, 48),
        (52, 199, 89),
        (0, 122, 255),
        (255, 149, 0),
        (175, 82, 222),
        (255, 45, 85),
    ]
    return palette[path_id % len(palette)]


def draw_cross(draw: ImageDraw.ImageDraw, center: tuple[float, float], color: tuple[int, int, int], size: int = 4) -> None:
    x, y = center
    draw.line((x - size, y, x + size, y), fill=color, width=1)
    draw.line((x, y - size, x, y + size), fill=color, width=1)


def write_preview_image(
    image_path: Path,
    preview_path: Path,
    paths: Sequence[np.ndarray],
) -> None:
    image = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(image)
    for path_id, path in enumerate(paths):
        color = path_color(path_id)
        polyline = [(float(point[1]), float(point[0])) for point in path]
        if len(polyline) >= 2:
            draw.line(polyline, fill=color, width=3)

        start = polyline[0]
        end = polyline[-1]
        draw_cross(draw, start, (255, 0, 0), size=6)
        draw_cross(draw, end, (255, 196, 0), size=6)
        draw.text((start[0] + 6, start[1] - 10), str(path_id), fill=color)

    image.save(preview_path)


def fmt_move(code: str, *, x: float | None = None, y: float | None = None, z: float | None = None, feedrate: float | None = None) -> str:
    parts = [code]
    if x is not None:
        parts.append(f"X{x:.3f}")
    if y is not None:
        parts.append(f"Y{y:.3f}")
    if z is not None:
        parts.append(f"Z{z:.3f}")
    if feedrate is not None:
        parts.append(f"F{feedrate:.0f}")
    return " ".join(parts)


def generate_gcode(
    paths: Sequence[np.ndarray],
    travel_z_mm: float,
    draw_z_mm: float,
    feedrate: float,
    close_loops: bool,
) -> str:
    lines = [DEFAULT_HEADER.rstrip()]
    current = np.array([0.0, 0.0], dtype=float)
    pen_is_down = True

    for path in paths:
        start = path[0]
        delta_to_start = start - current
        if pen_is_down:
            lines.append(fmt_move("G1", z=travel_z_mm, feedrate=feedrate))
            pen_is_down = False
        lines.append(
            fmt_move("G0", x=delta_to_start[0], y=delta_to_start[1])
        )
        lines.append(fmt_move("G1", z=draw_z_mm, feedrate=feedrate))
        pen_is_down = True

        previous = start
        for point in path[1:]:
            delta = point - previous
            lines.append(fmt_move("G1", x=delta[0], y=delta[1], feedrate=feedrate))
            previous = point

        if close_loops and len(path) > 2:
            delta = start - previous
            if np.linalg.norm(delta) > 1e-6:
                lines.append(fmt_move("G1", x=delta[0], y=delta[1], feedrate=feedrate))
                previous = start

        current = previous

    if pen_is_down:
        lines.append(fmt_move("G1", z=travel_z_mm, feedrate=feedrate))
    lines.append(DEFAULT_FOOTER.strip())
    return "\n".join(lines) + "\n"


def main() -> None:
    config = parse_args()
    mask = load_mask(config.image_path, config.threshold, config.min_area_px)
    raw_paths = extract_paths(mask, config.simplify_tol_px)
    if not raw_paths:
        raise SystemExit(
            "No se encontraron contornos. Ajusta --threshold o --min-area-px."
        )

    scaled_paths = scale_paths(
        raw_paths,
        canvas_width_mm=config.canvas_width_mm,
        canvas_height_mm=config.canvas_height_mm,
        start_x_mm=config.start_x_mm,
        start_y_mm=config.start_y_mm,
    )

    write_points_csv(config.points_path, scaled_paths)
    write_preview_image(config.image_path, config.preview_path, raw_paths)
    gcode = generate_gcode(
        scaled_paths,
        travel_z_mm=config.travel_z_mm,
        draw_z_mm=config.draw_z_mm,
        feedrate=config.feedrate,
        close_loops=config.close_loops,
    )
    config.gcode_path.write_text(gcode, encoding="utf-8")

    total_paths = len(scaled_paths)
    total_points = sum(len(path) for path in scaled_paths)
    print(f"Contornos detectados: {total_paths}")
    print(f"Puntos generados: {total_points}")
    print(f"CSV: {config.points_path}")
    print(f"Preview: {config.preview_path}")
    print(f"G-code: {config.gcode_path}")


if __name__ == "__main__":
    main()
