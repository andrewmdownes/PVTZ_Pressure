import tkinter as tk
from tkinter import filedialog
import csv

class CSVApp:
    def __init__(self, root):
        self.root = root
        self.root.title("CSV File Creator")
        self.root.geometry("400x600")

        # Create widgets
        self.start_button = tk.Button(root, text="Start", command=self.start)
        self.start_button.grid(row=0, column=0, columnspan=2, padx=10, pady=10)

        self.csv_name_label = tk.Label(root, text="Enter CSV file name:")
        self.csv_name_label.grid(row=1, column=0, columnspan=2, padx=10, pady=10)

        self.csv_name_entry = tk.Entry(root)
        self.csv_name_entry.grid(row=2, column=0, columnspan=2, padx=10, pady=10)

        self.csv_location_button = tk.Button(root, text="Select CSV Location", command=self.select_location)
        self.csv_location_button.grid(row=3, column=0, columnspan=2, padx=10, pady=10)

        self.create_csv_button = tk.Button(root, text="Create CSV", command=self.create_csv)
        self.create_csv_button.grid(row=4, column=0, columnspan=2, padx=10, pady=10)

        self.power_off_button = tk.Button(root, text="Total Power Off", command=self.power_off)
        self.power_off_button.grid(row=5, column=0, columnspan=2, padx=10, pady=10)

        self.close_button = tk.Button(root, text="Close Window", command=root.quit)
        self.close_button.grid(row=6, column=0, columnspan=2, padx=10, pady=10)

        # Initialize variables
        self.csv_location = ""

    def start(self):
        print("Application started!")

    def select_location(self):
        self.csv_location = filedialog.askdirectory()
        print(f"CSV location set to: {self.csv_location}")

    def create_csv(self):
        csv_name = self.csv_name_entry.get()
        if not csv_name:
            print("Please enter a valid CSV file name.")
            return

        if not self.csv_location:
            print("Please select a CSV file location.")
            return

        # Create a sample CSV file (you can modify this part)
        sample_data = [["Name", "Age"], ["Alice", 25], ["Bob", 30]]
        csv_file_path = f"{self.csv_location}/{csv_name}.csv"

        with open(csv_file_path, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(sample_data)

        print(f"CSV file created at {csv_file_path}")

    def power_off(self):
        print("All Power Off")

if __name__ == "__main__":
    root = tk.Tk()
    CSVApp(root)
    root.mainloop()
