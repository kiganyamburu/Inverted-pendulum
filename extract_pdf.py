import pdfplumber
from PIL import Image
import io

with pdfplumber.open("inverted pendulum.pdf") as pdf:
    print(f"Total pages: {len(pdf.pages)}\n")
    for i, page in enumerate(pdf.pages):
        print(f"\n{'='*80}")
        print(f"PAGE {i+1}")
        print(f"{'='*80}\n")
        text = page.extract_text()
        if text:
            print(text)
        else:
            print("[No text extracted from this page]")

        # Extract images
        if page.images:
            print(f"\n[This page contains {len(page.images)} image(s)]")
            # Convert page to image
            img = page.to_image(resolution=300)
            img.save(f"page_{i+1}.png")
