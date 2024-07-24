from python_qt_binding.QtWidgets import (
    QLineEdit, 
    QDialog,
    QApplication,
    QDialogButtonBox, 
    QFormLayout
)

class ROSDialog(QDialog):
    def __init__(self, label:str, topic:str):
        super(ROSDialog, self).__init__()
        buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self)
        layout = QFormLayout(self)
        self.setWindowTitle(topic)        

        self.inputs = []
        self.line_label = QLineEdit(self)
        self.line_hz = QLineEdit(self)

        self.inputs.append(self.line_label)
        layout.addRow("label", self.inputs[-1])
        self.line_label.setText(label)
        self.inputs.append(self.line_hz)
        layout.addRow("hz", self.inputs[-1])
        self.line_hz.setText("")

        layout.addWidget(buttonBox)

        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)


    def getInputs(self):
        return tuple(input.text() for input in self.inputs)

