"""Собирает docs/screens/demo.gif и all.jpg из картинок в docs/screens.

Запуск из папки проекта:
    python tools/make_gallery.py

Добавить новый кадр: положи файл .jpg/.png в docs/screens/ и запусти скрипт.
Файлы demo.gif и all.jpg не включаются в галерею.
"""
from pathlib import Path
from PIL import Image, ImageOps, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
SCREENS = ROOT / "docs" / "screens"
EXCLUDE = {"demo.gif", "all.jpg"}

CAPTIONS = {
    "01-device-with-model.jpg": "Весы с моделью",
    "02-device-weights.jpg": "Железо и гири",
    "03-screen-calibration.jpg": "Экран: калибровка датчика",
    "04-render.jpg": "3D-модель корпуса",
    "05-mechanics.png": "Схема геометрии опор",
}

W, H, BAR = 900, 600, 54


def font(size=28):
    for path in (r"C:\Windows\Fonts\arial.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            pass
    return ImageFont.load_default()


def caption(name: str) -> str:
    return CAPTIONS.get(name, Path(name).stem)


def main():
    files = sorted(
        p for p in SCREENS.iterdir()
        if p.suffix.lower() in {".jpg", ".jpeg", ".png"} and p.name not in EXCLUDE
    )
    if not files:
        raise SystemExit("Нет картинок в docs/screens/")

    fnt = font()
    frames = []
    for p in files:
        im = ImageOps.exif_transpose(Image.open(p)).convert("RGB")
        im.thumbnail((W - 24, H - BAR - 20), Image.LANCZOS)
        frame = Image.new("RGB", (W, H), (14, 17, 22))
        frame.paste(im, ((W - im.width) // 2, (H - BAR - im.height) // 2))
        d = ImageDraw.Draw(frame)
        text = caption(p.name)
        tw = d.textlength(text, font=fnt)
        d.text(((W - tw) / 2, H - BAR + 12), text, fill=(235, 240, 248), font=fnt)
        frames.append(frame)

    gif = SCREENS / "demo.gif"
    frames[0].save(gif, save_all=True, append_images=frames[1:],
                   duration=2600, loop=0, optimize=True)

    # Статичная сводка для быстрого просмотра.
    cols, tw, th = 2, 640, 440
    rows = (len(files) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * tw, rows * th), (14, 17, 22))
    for i, p in enumerate(files):
        im = ImageOps.exif_transpose(Image.open(p)).convert("RGB")
        im.thumbnail((tw - 20, th - 60), Image.LANCZOS)
        x = (i % cols) * tw + (tw - im.width) // 2
        y = (i // cols) * th + 10
        sheet.paste(im, (x, y))
        d = ImageDraw.Draw(sheet)
        text = caption(p.name)
        cw = d.textlength(text, font=fnt)
        d.text(((i % cols) * tw + (tw - cw) / 2, (i // cols) * th + th - 44),
               text, fill=(235, 240, 248), font=fnt)
    sheet.save(SCREENS / "all.jpg", quality=88, optimize=True)

    print(f"Готово: {gif} ({gif.stat().st_size // 1024} КБ), кадров: {len(files)}")


if __name__ == "__main__":
    main()
