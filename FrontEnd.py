import argparse
import sys
import BackEnd
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGridLayout, QFrame, QComboBox, QLineEdit
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
        # Make a vertical layout for the buttons
        button_layout = QVBoxLayout()

        # Adding plot OD dropdown menu and button
        self.od_dropdown = QComboBox(self)
        self.od_dropdown.addItems(BackEnd.populate_dropdown())
        self.OD_button = QPushButton('Plot OD', self)
        self.stats_button = QPushButton('Update Stats', self)

        # Input field for time scale in hours
        self.start_time_input = QLineEdit(self)
        self.start_time_input.setPlaceholderText("Enter start time in hours")

        self.end_time_input = QLineEdit(self)
        self.end_time_input.setPlaceholderText("Enter end time in hours")
        
        # Button Functionality
        self.OD_button.clicked.connect(self.OD_clicked)
        self.stats_button.clicked.connect(self.stats_clicked)

        # Button layout
        button_layout.addWidget(self.od_dropdown)
        button_layout.addWidget(self.OD_button)
        button_layout.addWidget(self.stats_button)
        button_layout.addWidget(QLabel("Time Scale (hours):", self))
        button_layout.addWidget(self.start_time_input)
        button_layout.addWidget(self.end_time_input)
        button_layout.addStretch()

        self.button_layout = button_layout

    def initDisplay(self):
        # Create a grid layout for the labels inside a frame
        frame = QFrame(self)
        frame.setFrameShape(QFrame.Box)
        frame.setLineWidth(2)
        
        grid_layout = QGridLayout(frame)
        
        # Create labels
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

        # Add labels to the grid layout
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
        # Create a layout for the plot
        self.plot_layout = QVBoxLayout()
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumSize(QSize(500, 300))  # Set the minimum size of the canvas

        # Add the canvas and toolbar to the layout
        self.plot_layout.addWidget(self.canvas)

        # Add the matplotlib toolbar
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        self.plot_layout.addWidget(self.toolbar)

        # Set the initial titles for the axes
        self.ax.set_xlabel('Time (hours)')
        self.ax.set_ylabel('OD940')
        self.figure.tight_layout(pad=3)

    def initUI(self):
        # Initialize main layout
        main_layout = QHBoxLayout()

        # Add button layout and frame layout to the main layout
        main_layout.addLayout(self.button_layout)
        main_layout.addWidget(self.frame)
        main_layout.addLayout(self.plot_layout)

        # Set the layout for the main window
        self.setLayout(main_layout)

        # Window settings
        self.setWindowTitle('easyEVO')
        self.setGeometry(300, 300, 600, 400)

        self.show()
    
    def OD_clicked(self):
        experiment_num = self.od_dropdown.currentIndex()
        start_time_text = self.start_time_input.text()
        end_time_text = self.end_time_input.text()

        self.ax.clear()

        # Validate start and end times
        try:
            start_time_hours = float(start_time_text) if start_time_text.strip() else None
            end_time_hours = float(end_time_text) if end_time_text.strip() else None

            # Ensure start_time is less than end_time if both are provided
            if start_time_hours is not None and end_time_hours is not None:
                if start_time_hours >= end_time_hours:
                    print("Start time must be less than end time.")
                    return
        except ValueError:
            print("Invalid time input. Please enter valid numbers.")
            return

        # Call the plotting function with the validated start and end times
        BackEnd.plot_OD(self.ax, experiment_num, start_time_hours, end_time_hours)
        self.ax.set_xlabel('Time (hours)')
        self.canvas.draw()


    def stats_clicked(self):
        last_row = BackEnd.read_stats()
        self.uptime_display.setText(f"{last_row['upTime']}")
        self.ambient_temp_display.setText(f"{last_row['ambientTemp']}°C")
        self.media_temp_display.setText(f"{last_row['mediaTemp']}°C")
        self.heaterplate_temp_display.setText(f"{last_row['heaterPlateTemp']}°C")
        self.ir_display.setText(f"{last_row['infraredReading']}")
        self.od_display.setText(f"{round(last_row['OD940'], 4)}")

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
