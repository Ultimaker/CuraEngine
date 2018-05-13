import matplotlib
import sys
import os
from helper.data_file_helper import DataFileHelper
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

matplotlib.use("TkAgg")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


if 2 == sys.version_info[0]:
    import ttk as TTK
    import Tkinter as TK
    from Tkinter import *
    from ttk import *
    import tkFileDialog as FD
    text = unicode
else:
    import tkinter.ttk as TTK
    import tkinter as TK
    from tkinter import *
    from tkinter.ttk import *
    from tkinter import filedialog as FD
    text = str


class MainApp:
    DEFAULT_FILE_PATH = '/tmp/parts_by_layers.txt'

    def __init__(self):
        # initialized variables
        self._storage = None
        if os.path.isfile(MainApp.DEFAULT_FILE_PATH):
            self._file_path = MainApp.DEFAULT_FILE_PATH
        else:
            self._file_path = None
        self._layer_index = None
        self.file_path_lable = None
        # create widgets
        self._create_widgets()
        # load data by default file path
        self.load_parts_data(self._file_path)
        self.refresh_graph()

    def _create_widgets(self):
        # create master
        self.master = Tk()
        self.master.title('Parts Viewer')
        # hook up window delete event
        self.master.protocol("WM_DELETE_WINDOW", self.quit)

        # create widgets
        self._create_panel_main()

        # fill in screen
        # w = self.master.winfo_screenwidth()
        # h = self.master.winfo_screenheight()
        # self.master.geometry("%dx%d+0+0" % (w, h))

    def _create_panel_main(self):
        self.file_panel = Frame(self.master, name="file")
        self.file_panel.grid(row=0, column=0)

        a_button = Button(self.file_panel, text='Open parts file...', command=self.ask_parts_file_dialog)
        a_button.grid(row=0, column=0, padx=20, pady=5, sticky=W)
        self.file_path_label_text = StringVar()
        self.file_path_label_text.set(self._file_path)
        self.file_path_lable = Label(self.file_panel, textvariable=self.file_path_label_text)
        self.file_path_lable.grid(row=0, column=1, sticky=E+W)

        ### plot
        self.figure = Figure()
        self.ax = self.figure.add_subplot(111)
        # self.line, = self.ax.plot(range(10))

        self.canvas = FigureCanvasTkAgg(self.figure, master=self.master)
        self.canvas.show()
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky=N+S, padx=5, pady=5)

        # grid layout
        self.master.columnconfigure(0, weight=1)
        self.master.rowconfigure(1, weight=1)

        # scale
        self.selected_layer_index = IntVar()
        self.scale = TK.Scale(self.master, from_=1, to=255, tickinterval=0, variable=self.selected_layer_index,
                              label='Layer Index', orient="vertical", command=self.update_layer_index)

        self.scale.grid(row=1, column=1, sticky=N+S, padx=20, pady=30)

    def ask_parts_file_dialog(self):
        file_path = FD.askopenfilename()
        if file_path is not None and len(file_path) != 0:
            self._update_file_path(file_path)
            self.load_parts_data(file_path)
            self.refresh_graph()
        return file_path

    def load_parts_data(self, file_path):
        if os.path.isfile(file_path):
            self._storage, msg = DataFileHelper.load_part_file(file_path)
            self.scale.config(to=self._storage.get_volume(0).get_layer_size())

    def refresh_graph(self, layer_index=0):
        parts = self._storage.get_volume(0).get_layer(layer_index)
        self.ax.clear()
        all_x = list()
        all_y = list()
        lines = list()
        for part_nr in range(parts.get_parts_size()):
            for outline_nr in range(parts.get_part(part_nr).get_outlines_size()):
                x = parts.get_part(part_nr).get_outline(outline_nr).get_all_points_at_x()
                y = parts.get_part(part_nr).get_outline(outline_nr).get_all_points_at_y()
                x = [e/1000 for e in x]
                y = [e/1000 for e in y]
                all_x = all_x + x
                all_y = all_y + y
                # end point connects to begin point
                x.append(x[0])
                y.append(y[0])
                lines.append(Line2D(x, y))

        for line in lines:
            self.ax.add_line(line)

        self.ax.set_xlim(0, max(all_x))
        self.ax.set_ylim(0, max(all_y))
        plt.gca().set_aspect('equal', adjustable='box')
        self.canvas.draw()

    def update_layer_index(self, value):
        if self._storage is not None:
            self.refresh_graph(self.selected_layer_index.get() - 1)

    def _update_file_path(self, file_path):
        self._file_path = file_path
        self.file_path_label_text.set(file_path)

    def quit(self):
        self.master.destroy()

    def main_loop(self):
        self.master.mainloop()


if __name__ == '__main__':
    app = MainApp()
    app.main_loop()
