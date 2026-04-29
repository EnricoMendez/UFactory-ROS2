# How to run `route_gen.py`

This script converts an input image into detected contours, scales those contours to millimeters, and generates:

- a `.gcode` file for drawing/plotting,
- a `.csv` file with the generated path points,
- a preview image showing the detected contours.

## 1. Install dependencies

Download the script: from [source](https://github.com/EnricoMendez/UFactory-ROS2/blob/main/codes/route_gen.py)

Or from bash

``` bash
wget https://raw.githubusercontent.com/EnricoMendez/UFactory-ROS2/main/codes/route_gen.py
```

Create and activate a Python environment if needed:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

Install the required Python packages:

```bash
pip install numpy pillow scikit-image
```

## 2. Place the input image

Put your input image in the same folder as `route_gen.py`.

By default, the script expects:

```bash
airlab_logo.png
```

## 3. Run with default parameters

```bash
python3 route_gen.py
```

This creates:

```text
logo.gcode
logo_points.csv
logo_contours.png
```

## 4. Run with a custom image and output files

```bash
python3 route_gen.py \
  --image airlab_logo.png \
  --gcode-out logo.gcode \
  --points-out logo_points.csv \
  --preview-out logo_contours.png
```

## 5. Common parameter adjustments

### Change drawing size

```bash
python3 route_gen.py \
  --image airlab_logo.png \
  --canvas-width-mm 400 \
  --canvas-height-mm 180
```

### Adjust contour detection

Use this if the script detects too many or too few contours:

```bash
python3 route_gen.py \
  --image airlab_logo.png \
  --threshold 245 \
  --min-area-px 300 \
  --simplify-tol-px 1.5
```

Meaning:

- `--threshold`: pixels darker than this value are treated as drawing.
- `--min-area-px`: removes small detected regions/noise.
- `--simplify-tol-px`: simplifies contours; higher values generate fewer points.

### Adjust Z positions and feedrate

```bash
python3 route_gen.py \
  --image airlab_logo.png \
  --travel-z-mm 20 \
  --draw-z-mm -20 \
  --feedrate 5000
```

Meaning:

- `--travel-z-mm`: Z height when the tool/pen is lifted.
- `--draw-z-mm`: Z height used while drawing.
- `--feedrate`: movement speed for `G1` commands.

## 6. Example full command

```bash
python3 route_gen.py \
  --image airlab_logo.png \
  --gcode-out logo.gcode \
  --points-out logo_points.csv \
  --preview-out logo_contours.png \
  --threshold 245 \
  --min-area-px 300 \
  --simplify-tol-px 1.5 \
  --canvas-width-mm 400 \
  --canvas-height-mm 180 \
  --travel-z-mm 20 \
  --draw-z-mm -20 \
  --feedrate 5000
```

## 7. Output check

After running, the terminal should show something like:

```text
Contornos detectados: 10
Puntos generados: 350
CSV: logo_points.csv
Preview: logo_contours.png
G-code: logo.gcode
```

Open `logo_contours.png` first to verify that the detected paths look correct before using the generated G-code.

## 8. Executing path

Copy the content from the `.gcode` file and paste it in the gcode ufactory interace. Then simulate the path, check there are no visible collisions and the run your code.

## 9. Troubleshooting

### No contours found

If you get:

```text
No se encontraron contornos. Ajusta --threshold o --min-area-px.
```

Try lowering `--min-area-px` or changing `--threshold`:

```bash
python3 route_gen.py --threshold 230 --min-area-px 80
```

### Too much noise

Increase `--min-area-px`:

```bash
python3 route_gen.py --min-area-px 500
```

### Too many points in the G-code

Increase `--simplify-tol-px`:

```bash
python3 route_gen.py --simplify-tol-px 3.0
```

### Drawing too large or too small

Adjust the canvas size:

```bash
python3 route_gen.py --canvas-width-mm 300 --canvas-height-mm 120
```
