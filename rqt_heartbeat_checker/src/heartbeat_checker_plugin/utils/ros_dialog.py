from python_qt_binding.QtWidgets import (
    QLineEdit, 
    QDialog,
    QDialogButtonBox, 
    QFormLayout,
)
from python_qt_binding.QtGui import QDoubleValidator

class ROSDialog(QDialog):
    def __init__(self, label:str, topic:str, hz:float=1):
        super(ROSDialog, self).__init__()
        # set the time of the menu window as topic name
        self.setWindowTitle(topic)

        # set the button box to click ok or cancel
        buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self)
        
        # basic layout of menu window
        layout = QFormLayout(self)

        # QLineEdit for able entering the label name, hz
        self.line_label = QLineEdit(self)
        self.line_hz = QLineEdit(self)
        
        # set input and label
        self.inputs = []
        self.inputs.append(self.line_label)
        layout.addRow("label ", self.inputs[-1])
        self.inputs.append(self.line_hz)
        layout.addRow("hz", self.inputs[-1])

        # add button box at the bottom of the menu window
        layout.addWidget(buttonBox)

        # set button box its role
        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)

        # insert the original value of each label
        # insert original label name
        self.line_label.setText(label)
        # insert original hz
        self.line_hz.setText(str(hz))

    def getInputs(self):
        return tuple(input.text() for input in self.inputs)
    
    def __del__(self):
        pass