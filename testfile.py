from PyQt4.QtGui import *
from PyQt4.QtCore import *

import sys

class myListWidget(QListWidget):

    def Clicked(self,item):
        QMessageBox.information(self, "ListWidget", "You clicked: "+item.text())
        
class App(QWidget):

    def __init__(self):
        super(App,self).__init__()
        
        grid = QGridLayout()

        listWidget = myListWidget(self)
        
        grid.addWidget(listWidget,0,0)
        #Resize width and height
        listWidget.resize(300,120)
        
        listWidget.addItem("Item 1"); 
        listWidget.addItem("Item 2");
        listWidget.addItem("Item 3");
        listWidget.addItem("Item 4");
        
        listWidget.itemClicked.connect(listWidget.Clicked)
       

        self.setWindowTitle("Brillouin Scan Interface")
        self.move(50,50)
        self.show()
    
if __name__ == '__main__':

    app = QApplication(sys.argv)
    gui = App()
    sys.exit(app.exec_())