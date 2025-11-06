#!/usr/bin/env python3
"""
EXIF Data Stripper - GUI Version

Simple GUI tool to strip EXIF data from images in a folder.
Just run it, click Browse, select your folder, and click Strip EXIF!
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
from pathlib import Path
from PIL import Image
import threading


class ExifStripperGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        
        self.title("EXIF Data Stripper")
        self.geometry("600x500")
        self.resizable(True, True)
        
        # Variables
        self.folder_path = tk.StringVar()
        self.recursive_var = tk.BooleanVar(value=False)
        self.backup_var = tk.BooleanVar(value=True)
        self.processing = False
        
        # Build GUI
        self._build_gui()
        
    def _build_gui(self):
        # Main frame with padding
        main_frame = ttk.Frame(self, padding=10)
        main_frame.pack(fill="both", expand=True)
        
        # Title
        title_label = ttk.Label(main_frame, text="EXIF Data Stripper", 
                               font=("TkDefaultFont", 16, "bold"))
        title_label.pack(pady=(0, 10))
        
        # Folder selection frame
        folder_frame = ttk.LabelFrame(main_frame, text="Select Folder", padding=10)
        folder_frame.pack(fill="x", pady=(0, 10))
        
        folder_entry_frame = ttk.Frame(folder_frame)
        folder_entry_frame.pack(fill="x")
        
        self.folder_entry = ttk.Entry(folder_entry_frame, textvariable=self.folder_path)
        self.folder_entry.pack(side="left", fill="x", expand=True, padx=(0, 5))
        
        browse_btn = ttk.Button(folder_entry_frame, text="Browse...", 
                               command=self.browse_folder)
        browse_btn.pack(side="right")
        
        # Options frame
        options_frame = ttk.LabelFrame(main_frame, text="Options", padding=10)
        options_frame.pack(fill="x", pady=(0, 10))
        
        recursive_check = ttk.Checkbutton(options_frame, 
                                          text="Include subdirectories (recursive)",
                                          variable=self.recursive_var)
        recursive_check.pack(anchor="w")
        
        backup_check = ttk.Checkbutton(options_frame, 
                                       text="Create backup files (.original)",
                                       variable=self.backup_var)
        backup_check.pack(anchor="w")
        
        # Action button
        self.strip_btn = ttk.Button(main_frame, text="Strip EXIF Data", 
                                    command=self.start_stripping)
        self.strip_btn.pack(pady=(0, 10))
        
        # Progress frame
        progress_frame = ttk.LabelFrame(main_frame, text="Progress", padding=10)
        progress_frame.pack(fill="both", expand=True)
        
        self.progress_text = scrolledtext.ScrolledText(progress_frame, 
                                                       height=15, width=70,
                                                       state="disabled")
        self.progress_text.pack(fill="both", expand=True)
        
        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(self, textvariable=self.status_var, 
                              relief="sunken", anchor="w")
        status_bar.pack(side="bottom", fill="x")
        
    def browse_folder(self):
        """Open folder browser dialog."""
        folder = filedialog.askdirectory(title="Select Folder with Images")
        if folder:
            self.folder_path.set(folder)
            
    def log(self, message):
        """Add message to progress log."""
        self.progress_text.config(state="normal")
        self.progress_text.insert("end", message + "\n")
        self.progress_text.see("end")
        self.progress_text.config(state="disabled")
        self.update_idletasks()
        
    def strip_exif_from_image(self, image_path: Path, create_backup: bool) -> bool:
        """Strip EXIF data from a single image."""
        try:
            img = Image.open(image_path)
            
            if create_backup:
                backup_path = image_path.with_suffix(image_path.suffix + '.original')
                if not backup_path.exists():
                    img.save(backup_path)
                    self.log(f"  ✓ Backup: {backup_path.name}")
            
            data = list(img.getdata())
            image_without_exif = Image.new(img.mode, img.size)
            image_without_exif.putdata(data)
            
            if image_path.suffix.lower() in {'.jpg', '.jpeg'}:
                image_without_exif.save(image_path, "JPEG", quality=95, optimize=True)
            elif image_path.suffix.lower() == '.png':
                image_without_exif.save(image_path, "PNG", optimize=True)
            elif image_path.suffix.lower() in {'.tiff', '.tif'}:
                image_without_exif.save(image_path, "TIFF")
            else:
                image_without_exif.save(image_path)
            
            self.log(f"  ✓ Stripped: {image_path.name}")
            return True
            
        except Exception as e:
            self.log(f"  ✗ Error: {image_path.name} - {e}")
            return False
            
    def process_folder(self):
        """Process all images in selected folder."""
        folder = Path(self.folder_path.get())
        
        if not folder.exists():
            messagebox.showerror("Error", "Selected folder does not exist!")
            return
            
        self.log("=" * 60)
        self.log(f"Processing folder: {folder}")
        self.log(f"Recursive: {self.recursive_var.get()}")
        self.log(f"Create backups: {self.backup_var.get()}")
        self.log("=" * 60)
        self.log("")
        
        # Find images
        image_extensions = {'.jpg', '.jpeg', '.png', '.tiff', '.tif'}
        
        if self.recursive_var.get():
            image_files = [f for f in folder.rglob('*') 
                          if f.is_file() and f.suffix.lower() in image_extensions]
        else:
            image_files = [f for f in folder.iterdir() 
                          if f.is_file() and f.suffix.lower() in image_extensions]
        
        if not image_files:
            self.log("No image files found!")
            messagebox.showinfo("Info", "No image files found in selected folder.")
            return
            
        self.log(f"Found {len(image_files)} image(s)\n")
        
        # Process images
        success_count = 0
        fail_count = 0
        
        for i, image_file in enumerate(sorted(image_files), 1):
            self.log(f"[{i}/{len(image_files)}] {image_file.name}")
            self.status_var.set(f"Processing {i}/{len(image_files)}: {image_file.name}")
            
            if self.strip_exif_from_image(image_file, self.backup_var.get()):
                success_count += 1
            else:
                fail_count += 1
            self.log("")
        
        # Results
        self.log("=" * 60)
        self.log(f"Results:")
        self.log(f"  Successfully processed: {success_count}")
        self.log(f"  Failed: {fail_count}")
        self.log(f"  Total: {success_count + fail_count}")
        self.log("=" * 60)
        
        self.status_var.set(f"Done! Processed {success_count} images")
        
        messagebox.showinfo("Complete", 
                           f"EXIF stripping complete!\n\n"
                           f"Successfully processed: {success_count}\n"
                           f"Failed: {fail_count}")
        
    def start_stripping(self):
        """Start the EXIF stripping process in a thread."""
        if not self.folder_path.get():
            messagebox.showwarning("Warning", "Please select a folder first!")
            return
            
        if self.processing:
            messagebox.showwarning("Warning", "Already processing!")
            return
            
        # Confirm
        response = messagebox.askyesno("Confirm", 
                                      "Strip EXIF data from all images in selected folder?\n\n"
                                      "Backups will be created if the option is enabled.")
        if not response:
            return
            
        # Clear log
        self.progress_text.config(state="normal")
        self.progress_text.delete(1.0, "end")
        self.progress_text.config(state="disabled")
        
        # Disable button
        self.processing = True
        self.strip_btn.config(state="disabled")
        
        # Process in thread
        def process_thread():
            try:
                self.process_folder()
            finally:
                self.processing = False
                self.strip_btn.config(state="normal")
                
        thread = threading.Thread(target=process_thread, daemon=True)
        thread.start()


if __name__ == "__main__":
    app = ExifStripperGUI()
    app.mainloop()
