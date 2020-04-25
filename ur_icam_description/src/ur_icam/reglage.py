# coding: utf-8
from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5.QtCore import Qt

class Reglage(QWidget):
    def __init__(self,min,max,text,val=0):
        super(Reglage,self).__init__()
        #loadUi('reglage.ui', self)
        # Définition de l'interface
        hLayout = QHBoxLayout()
        self.label = QLabel()
        hLayout.addWidget(self.label)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMaximum(9999)
        hLayout.addWidget(self.slider)
        self.edit = QLineEdit()
        self.edit.setMaxLength(6)
        hLayout.addWidget(self.edit)
        # initialisation
        self.min = min
        self.max = max
        self.a = (max - min) / 9999.0  # Coeff de valeur = a * slidder + b (avec minSlidder = 0 et maxSlidder = 9999)
        self.b = min
        self.slider.setSliderPosition(val)
        self.label.setText(text)
        self.edit.setText(str(val))
        self.slider.setSliderPosition(self.invConvert(val))
        self.slider.valueChanged.connect(self.changerValeurEdit)
        self.edit.editingFinished.connect(self.changerValeurSlider)
        self.setLayout(hLayout)

    def value(self):
        return float(self.edit.text())

    def convert(self,pos):
        return pos * self.a + self.b

    def invConvert(self,val):
        return int((val - self.b) / float(self.a))

    def changerValeurEdit(self):
        val = self.slider.value()
        self.edit.setText('{:.3}'.format(self.convert(val)))

    def changerValeurSlider(self):
        try:
            val = float(self.edit.text())
            self.slider.setSliderPosition(self.invConvert(val))
        except ValueError:
            QMessageBox.error(self, "Error", "Must be a number")



if __name__ == '__main__':
      # Programme principal
      app = QApplication([])
      ihm = Reglage(-3.14,3.14,'tyty',1)
      ihm.show()
      app.exec_()

