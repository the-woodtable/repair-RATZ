import tkinter as tk

root = tk.Tk()
root.title("Test Window")
root.geometry("300x200")
label = tk.Label(root, text="Hello Farhan!", font=("Arial", 16))
label.pack(pady=50)
root.mainloop()
