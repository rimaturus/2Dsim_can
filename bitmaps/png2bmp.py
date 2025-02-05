import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk

class TransparentToPinkBMP:
    def __init__(self, master):
        self.master = master
        self.master.title("PNG to Pink-Background BMP")

        self.original_image = None
        self.preview_image = None

        # Build the simple GUI
        self.setup_gui()

    def setup_gui(self):
        # === Row: Open Button ===
        open_frame = tk.Frame(self.master)
        open_frame.pack(pady=5, fill=tk.X)

        open_btn = tk.Button(open_frame, text="Open PNG", command=self.open_png)
        open_btn.pack(side=tk.LEFT, padx=5)

        # === Label to show preview ===
        self.image_label = tk.Label(self.master, text="No image loaded")
        self.image_label.pack(pady=5)

        # === Row: Resize entries ===
        resize_frame = tk.Frame(self.master)
        resize_frame.pack(pady=5, fill=tk.X)

        tk.Label(resize_frame, text="New Width:").pack(side=tk.LEFT, padx=5)
        self.width_entry = tk.Entry(resize_frame, width=6)
        self.width_entry.pack(side=tk.LEFT)

        tk.Label(resize_frame, text="New Height:").pack(side=tk.LEFT, padx=5)
        self.height_entry = tk.Entry(resize_frame, width=6)
        self.height_entry.pack(side=tk.LEFT)

        # === Row: Save Button ===
        save_frame = tk.Frame(self.master)
        save_frame.pack(pady=5, fill=tk.X)

        save_btn = tk.Button(save_frame, text="Save as BMP", command=self.save_bmp)
        save_btn.pack(side=tk.LEFT, padx=5)

    def open_png(self):
        """Open a PNG, composite it onto pink, and show a preview."""
        file_path = filedialog.askopenfilename(
            filetypes=[("PNG Files", "*.png"), ("All Files", "*.*")]
        )
        if not file_path:
            return  # user canceled

        try:
            # 1) Read as RGBA to keep alpha channel
            img = Image.open(file_path).convert("RGBA")
        except Exception as e:
            messagebox.showerror("Error", f"Could not open image:\n{e}")
            return

        # 2) Create a pink background
        pink_bg = Image.new("RGBA", img.size, (255, 0, 255, 255))

        # 3) Alpha composite: place the PNG "over" the pink background
        composited = Image.alpha_composite(pink_bg, img)

        self.original_image = composited  # Store for later saving

        # 4) Generate a small thumbnail for preview
        preview_copy = composited.copy()
        preview_copy.thumbnail((300, 300))
        self.preview_image = ImageTk.PhotoImage(preview_copy)
        self.image_label.config(image=self.preview_image, text="")

        messagebox.showinfo("Info", "PNG loaded and composited onto pink background.")

    def save_bmp(self):
        """Resize (optional) and save as BMP (solid pink background, no transparency)."""
        if self.original_image is None:
            messagebox.showwarning("No Image", "No image to save.")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".bmp",
            filetypes=[("BMP Files", "*.bmp"), ("All Files", "*.*")]
        )
        if not file_path:
            return  # user canceled

        # Work on a copy
        out_img = self.original_image.copy().convert("RGBA")

        # Check if valid width/height were entered
        try:
            w = int(self.width_entry.get())
            h = int(self.height_entry.get())
            if w > 0 and h > 0:
                out_img = out_img.resize((w, h), Image.LANCZOS)
        except ValueError:
            # If invalid entries or empty, just skip resizing
            pass

        # BMP does not support alpha, so convert to RGB
        out_img = out_img.convert("RGB")

        try:
            out_img.save(file_path, "BMP")
            messagebox.showinfo("Success", f"Saved BMP: {file_path}")
        except Exception as e:
            messagebox.showerror("Error", f"Could not save BMP:\n{e}")

def main():
    root = tk.Tk()
    app = TransparentToPinkBMP(root)
    root.mainloop()

if __name__ == "__main__":
    main()
