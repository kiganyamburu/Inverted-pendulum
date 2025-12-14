from PIL import Image
import sys

# Open and display image info
img = Image.open("page_1.png")
print(f"Image size: {img.size}")
print(f"Image mode: {img.mode}")

# Try to extract text using OCR if available
try:
    import pytesseract

    text = pytesseract.image_to_string(img)
    print("\n" + "=" * 80)
    print("EXTRACTED TEXT FROM PDF IMAGE:")
    print("=" * 80)
    print(text)
except Exception as e:
    print(f"\nOCR not available: {e}")
    print("Tesseract may need to be installed separately")
