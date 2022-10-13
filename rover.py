import tkinter as tk
# Create the main window
root = tk.Tk()
root.title("Rover")
greet = tk.Label(text="running",
    fg="white",
    bg="red",
    width=20,
    height=10)
greet.pack()
print("point a")
root.mainloop()
print("point b")