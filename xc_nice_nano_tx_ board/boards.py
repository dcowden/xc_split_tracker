import argparse
import glob
import os
import shutil
import subprocess
import sys
from pathlib import Path


def find_kicad_cli(user_path: str | None) -> str:
    user_path = (user_path or "").strip().strip('"')
    if user_path:
        if Path(user_path).exists():
            return user_path
        w = shutil.which(user_path)
        if w:
            return w

    envp = os.environ.get("KICAD_CLI", "").strip().strip('"')
    if envp and Path(envp).exists():
        return envp

    w = shutil.which("kicad-cli")
    if w:
        return w

    # Common Windows installs
    if sys.platform.startswith("win"):
        candidates = [
            r"C:\Program Files\KiCad\9.0\bin\kicad-cli.exe",
            r"C:\Program Files\KiCad\8.0\bin\kicad-cli.exe",
            r"C:\Program Files\KiCad\7.0\bin\kicad-cli.exe",
        ]
        for p in candidates:
            if Path(p).exists():
                return p

    return ""


def find_magick(user_path: str | None) -> str:
    user_path = (user_path or "").strip().strip('"')
    if user_path:
        if Path(user_path).exists():
            return user_path
        w = shutil.which(user_path)
        if w:
            return w

    envp = os.environ.get("IM_MAGICK", "").strip().strip('"')
    if envp and Path(envp).exists():
        return envp

    w = shutil.which("magick")
    if w:
        return w

    if sys.platform.startswith("win"):
        # Not exhaustive; PATH is best. This is just a fallback.
        candidates = [
            r"C:\Program Files\ImageMagick-7.1.1-Q16-HDRI\magick.exe",
            r"C:\Program Files\ImageMagick-7.1.0-Q16-HDRI\magick.exe",
            r"C:\Program Files\ImageMagick-7.0.0-Q16\magick.exe",
        ]
        for p in candidates:
            if Path(p).exists():
                return p

    return ""


def run_cmd(cmd: list[str], log_lines: list[str], cwd: Path) -> None:
    # Hide pop-up consoles on Windows
    kwargs = dict(
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if sys.platform.startswith("win"):
        kwargs["creationflags"] = subprocess.CREATE_NO_WINDOW

    log_lines.append("RUN: " + " ".join(cmd))
    p = subprocess.run(cmd, **kwargs)
    if p.stdout:
        log_lines.append("STDOUT:\n" + p.stdout)
    if p.stderr:
        log_lines.append("STDERR:\n" + p.stderr)

    if p.returncode != 0:
        raise RuntimeError(f"Command failed (exit {p.returncode}): {' '.join(cmd)}")


def choose_pcb_file(explicit: str | None, folder: Path) -> Path:
    if explicit:
        p = (folder / explicit).resolve()
        if not p.exists():
            raise FileNotFoundError(f"PCB file not found: {p}")
        return p

    files = sorted(folder.glob("*.kicad_pcb"))
    if not files:
        raise FileNotFoundError(f"No .kicad_pcb found in: {folder}")

    if len(files) == 1:
        return files[0].resolve()

    # If multiple, choose newest (usually what you want)
    files.sort(key=lambda x: x.stat().st_mtime, reverse=True)
    return files[0].resolve()


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Generate board_art assets using kicad-cli + ImageMagick (batch-equivalent)."
    )
    ap.add_argument("--pcb", default=None, help="Optional .kicad_pcb filename. If omitted, auto-detects.")
    ap.add_argument("--dpi", type=int, default=1000, help="SVG->PNG raster DPI (matches your script default).")
    ap.add_argument("--kerf-px", type=int, default=8, help="Disk radius for dilation (matches your script default).")
    ap.add_argument("--kicad-cli", default=None, help="Path to kicad-cli (optional).")
    ap.add_argument("--magick", default=None, help="Path to ImageMagick magick.exe (optional).")
    ap.add_argument("--theme", default="KiCad Classic", help="KiCad theme name (default: KiCad Classic).")
    args = ap.parse_args()

    cwd = Path.cwd()
    pcb_path = choose_pcb_file(args.pcb, cwd)
    out_dir = cwd / "board_art2"
    out_dir.mkdir(parents=True, exist_ok=True)

    kicad_cli = find_kicad_cli(args.kicad_cli)
    if not kicad_cli:
        raise RuntimeError(
            "kicad-cli not found. Put it on PATH, or set KICAD_CLI, or pass --kicad-cli."
        )

    magick = find_magick(args.magick)
    if not magick:
        raise RuntimeError(
            "ImageMagick 'magick' not found. Put it on PATH, or set IM_MAGICK, or pass --magick."
        )

    dpi = str(args.dpi)
    kerf_px = str(args.kerf_px)
    theme = args.theme

    log_lines: list[str] = []
    log_lines.append(f"PCB: {pcb_path}")
    log_lines.append(f"OUT: {out_dir}")
    log_lines.append(f"DPI: {dpi}  KERF_PX: {kerf_px}")
    log_lines.append(f"kicad-cli: {kicad_cli}")
    log_lines.append(f"magick: {magick}")
    log_lines.append("")

    try:
        # --- Cuts DXF (exactly like your script) ---
        run_cmd(
            [
                kicad_cli, "pcb", "export", "dxf",
                "--output", str(out_dir / "cuts.dxf"),
                "--layers", "Edge.Cuts,User.Drawings",
                "--output-units", "mm",
                str(pcb_path),
            ],
            log_lines,
            cwd,
        )

        # Helper to export svg, rasterize, dilate
        def export_svg_png(base: str, flags: list[str], layers: str, do_dilate: bool) -> None:
            svg = out_dir / f"{base}.svg"
            png = out_dir / f"{base}.png"
            out_png = out_dir / (f"{base}_kerf.png" if do_dilate else f"{base}.png")

            # kicad-cli export svg (same flags/layers)
            cmd = [
                kicad_cli, "pcb", "export", "svg",
                "--output", str(svg),
                "--black-and-white",
                "--theme", theme,
                "--layers", layers,
                "--exclude-drawing-sheet",
                *flags,
                str(pcb_path),
            ]
            # Note: flags are appended plainly; readable and stable
            run_cmd(cmd, log_lines, cwd)

            # magick convert -density DPI svg -> png  (exact)
            run_cmd(
                [magick, "convert", "-density", dpi, str(svg), str(png)],
                log_lines,
                cwd,
            )

            # magick png -morphology Dilate Disk:KERF_PX png_kerf (exact)
            if do_dilate:
                run_cmd(
                    [magick, str(png), "-morphology", "Dilate", f"Disk:{kerf_px}", str(out_png)],
                    log_lines,
                    cwd,
                )

        # --- BACK (mirror) ---
        export_svg_png(
            base="back",
            flags=["--negative", "--mirror"],
            layers="B.Cu,Edge.Cuts",
            do_dilate=True,
        )
        export_svg_png(
            base="mask-back",
            flags=["--mirror"],
            layers="B.Mask,Edge.Cuts",
            do_dilate=True,
        )

        # --- FRONT (no mirror) ---
        export_svg_png(
            base="front",
            flags=["--negative"],
            layers="F.Cu,Edge.Cuts",
            do_dilate=True,
        )
        export_svg_png(
            base="mask-front",
            flags=[],
            layers="F.Mask,Edge.Cuts",
            do_dilate=True,
        )

        # --- Silks DXF (exactly like your script) ---
        run_cmd(
            [
                kicad_cli, "pcb", "export", "dxf",
                "--output", str(out_dir / "back-silkscreen.dxf"),
                "--layers", "B.Silkscreen,Edge.Cuts",
                "--output-units", "mm",
                str(pcb_path),
            ],
            log_lines,
            cwd,
        )
        run_cmd(
            [
                kicad_cli, "pcb", "export", "dxf",
                "--output", str(out_dir / "front-silkscreen.dxf"),
                "--layers", "F.Fab,Edge.Cuts",
                "--output-units", "mm",
                str(pcb_path),
            ],
            log_lines,
            cwd,
        )

        log_lines.append("\nDONE.")
        (out_dir / "run_log.txt").write_text("\n".join(log_lines), encoding="utf-8")
        return 0

    except Exception as e:
        log_lines.append("\nFAILED: " + repr(e))
        try:
            (out_dir / "run_log.txt").write_text("\n".join(log_lines), encoding="utf-8")
        except Exception:
            pass
        raise


if __name__ == "__main__":
    raise SystemExit(main())
