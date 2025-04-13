import argparse
import sys
import BackEnd
import plotter
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGridLayout, QFrame, QComboBox, QLineEdit
)
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT
from PyQt5.QtCore import QSize

class evoUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initButtons()
        self.initDisplay()
        self.initPlot()
        self.initUI()
            
    def initButtons(self):
        # Create a vertical layout for the buttons.
        button_layout = QVBoxLayout()

        # Dropdown for selecting a single experiment for plotting.
        self.od_dropdown = QComboBox(self)
        self.od_dropdown.addItems(BackEnd.populate_dropdown())
        self.OD_button = QPushButton('Plot OD', self)
        self.stats_button = QPushButton('Update Stats', self)

        # New: Input field for entering experiment numbers to merge.
        # Instruct the user to enter comma-separated experiment numbers (e.g., "1,3,5").
        self.merge_input = QLineEdit(self)
        self.merge_input.setPlaceholderText("Experiments to merge (e.g., 1,3,5)")

        # New: Button to merge experiments.
        self.merge_button = QPushButton('Merge Experiments', self)

        # Input fields for time scale in hours.
        self.start_time_input = QLineEdit(self)
        self.start_time_input.setPlaceholderText("Enter start time in hours")
        self.end_time_input = QLineEdit(self)
        self.end_time_input.setPlaceholderText("Enter end time in hours")
        
        # Connect buttons to their functions.
        self.OD_button.clicked.connect(self.OD_clicked)
        self.stats_button.clicked.connect(self.stats_clicked)
        self.merge_button.clicked.connect(self.merge_clicked)

        # Add all widgets to the button layout.
        button_layout.addWidget(self.od_dropdown)
        button_layout.addWidget(self.OD_button)
        button_layout.addWidget(self.stats_button)
        button_layout.addWidget(self.merge_input)     # Merge input field.
        button_layout.addWidget(self.merge_button)      # Merge experiments button.
        button_layout.addWidget(QLabel("Time Scale (hours):", self))
        button_layout.addWidget(self.start_time_input)
        button_layout.addWidget(self.end_time_input)
        button_layout.addStretch()

        self.button_layout = button_layout

    def initDisplay(self):
        # Create a frame for displaying status information.
        frame = QFrame(self)
        frame.setFrameShape(QFrame.Box)
        frame.setLineWidth(2)
        
        grid_layout = QGridLayout(frame)
        
        # Create labels for system statistics.
        self.uptime_label = QLabel('Uptime:', self)
        self.uptime_display = QLabel('0:00:00', self)
        
        self.ambient_temp_label = QLabel('Ambient Temperature:', self)
        self.ambient_temp_display = QLabel('0°C', self)
        
        self.media_temp_label = QLabel('Media Temperature:', self)
        self.media_temp_display = QLabel('0°C', self)
        
        self.heaterplate_temp_label = QLabel('HeaterPlate Temperature:', self)
        self.heaterplate_temp_display = QLabel('0°C', self)
        
        self.ir_label = QLabel('IR:', self)
        self.ir_display = QLabel('0', self)
        
        self.od_label = QLabel('OD:', self)
        self.od_display = QLabel('0', self)

        # Add labels to the grid layout.
        grid_layout.addWidget(self.uptime_label, 0, 0)
        grid_layout.addWidget(self.uptime_display, 0, 1)
        
        grid_layout.addWidget(self.ambient_temp_label, 1, 0)
        grid_layout.addWidget(self.ambient_temp_display, 1, 1)
        
        grid_layout.addWidget(self.media_temp_label, 2, 0)
        grid_layout.addWidget(self.media_temp_display, 2, 1)
        
        grid_layout.addWidget(self.heaterplate_temp_label, 3, 0)
        grid_layout.addWidget(self.heaterplate_temp_display, 3, 1)
        
        grid_layout.addWidget(self.ir_label, 4, 0)
        grid_layout.addWidget(self.ir_display, 4, 1)
        
        grid_layout.addWidget(self.od_label, 5, 0)
        grid_layout.addWidget(self.od_display, 5, 1)

        self.frame = frame

    def initPlot(self):
        # Create a layout for plotting.
        self.plot_layout = QVBoxLayout()
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumSize(QSize(500, 300))  # Set minimum canvas size.
        self.plot_layout.addWidget(self.canvas)

        # Add the matplotlib navigation toolbar.
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        self.plot_layout.addWidget(self.toolbar)

        # Set initial axes labels and adjust layout.
        self.ax.set_xlabel('Time (hours)')
        self.ax.set_ylabel('OD940')
        self.figure.tight_layout(pad=3)

    def initUI(self):
        # Combine the button layout, stats frame, and plot layout into the main layout.
        main_layout = QHBoxLayout()
        main_layout.addLayout(self.button_layout)
        main_layout.addWidget(self.frame)
        main_layout.addLayout(self.plot_layout)

        self.setLayout(main_layout)
        self.setWindowTitle('easyEVO')
        self.setGeometry(300, 300, 600, 400)
        self.show()
    
    def OD_clicked(self):
        """
        Plot a single experiment's OD data on the axis.
        """
        experiment_num = self.od_dropdown.currentIndex()
        start_time_text = self.start_time_input.text()
        end_time_text = self.end_time_input.text()
        self.ax.clear()

        # Validate time scale inputs.
        try:
            start_time_hours = float(start_time_text) if start_time_text.strip() else None
            end_time_hours = float(end_time_text) if end_time_text.strip() else None

            if start_time_hours is not None and end_time_hours is not None and start_time_hours >= end_time_hours:
                print("Start time must be less than end time.")
                return
        except ValueError:
            print("Invalid time input. Please enter valid numbers.")
            return

        # Use the backend function to plot the selected experiment.
        BackEnd.plot_OD(self.ax, experiment_num, start_time_hours, end_time_hours)
        self.ax.set_xlabel('Time (hours)')
        self.canvas.draw()

    def stats_clicked(self):
        """
        Update the displayed statistics using the latest available data.
        """
        last_row = BackEnd.read_stats()
        self.uptime_display.setText(f"{last_row['upTime']}")
        self.ambient_temp_display.setText(f"{last_row['ambientTemp']}°C")
        self.media_temp_display.setText(f"{last_row['mediaTemp']}°C")
        self.heaterplate_temp_display.setText(f"{last_row['heaterPlateTemp']}°C")
        self.ir_display.setText(f"{last_row['infraredReading']}")
        self.od_display.setText(f"{round(last_row['OD940'], 4)}")

    def merge_clicked(self):
        """
        Parse user-provided experiment numbers, merge the selected experiments (via plotter.merge_experiments),
        and plot the merged data on a continuous time axis.
        """
        self.ax.clear()
        merge_input_text = self.merge_input.text().strip()
        if merge_input_text == "":
            print("No experiments specified for merge; please enter experiment numbers (e.g., 1,3,5).")
            return

        # Parse the input into a list of zero-based indices (assuming user numbers are 1-based).
        try:
            selected_indices = [int(num.strip()) - 1 for num in merge_input_text.split(",") if num.strip()]
        except ValueError:
            print("Invalid experiment numbers. Please enter a comma-separated list of numbers (e.g., 1,3,5).")
            return

        # Call the merge_experiments function from the plotter module.
        merged_df = plotter.merge_experiments(selected_experiments=selected_indices)
        if merged_df is None:
            print("No merged experiments available for the specified selection.")
            return

        # Check for required columns and plot the merged data.
        # The 'upTime' column is converted from seconds to hours.
        if 'upTime' in merged_df.columns and ('OD940' in merged_df.columns or 'OD' in merged_df.columns):
            od_column = 'OD940' if 'OD940' in merged_df.columns else 'OD'
            self.ax.plot(merged_df['upTime'] / 3600.0, merged_df[od_column])
            self.ax.set_xlabel('Time (hours)')
            self.ax.set_ylabel(od_column)
            self.ax.set_title("Merged Experiment Data")
            self.canvas.draw()
        else:
            print("Required data columns ('upTime' and OD) not found in merged experiments.")

def main():
    parser = argparse.ArgumentParser(description='easyEVO GUI')
    parser.add_argument('--reset', action='store_true', help='Start new run')
    parser.add_argument('--continue', action='store_true', help='Resume old run')
    parser.add_argument('--testing', action='store_true', help='Skips serial connection')
    args = parser.parse_args()

    if args.testing:
        print('Testing Mode')
    elif args.reset:
        BackEnd.init_BackEnd_Connection(mode='reset')
    else:
        BackEnd.init_BackEnd_Connection(mode='continue')
        
    print("Initializing UI")
    app = QApplication(sys.argv)
    ex = evoUI()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
