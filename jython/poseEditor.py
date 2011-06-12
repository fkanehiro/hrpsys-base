import math
from javax.swing import *
from java.awt import *
import hrp
from OpenHRP.StateHolderServicePackage import CommandHolder

class jointPanel(JPanel):
    def __init__(self, linkinfo, seq, angle):
        self.seq = seq
        if len(linkinfo.ulimit) > 0:
            self.maxDeg = linkinfo.ulimit[0]*180/math.pi
        else:
            self.maxDeg = 180
        if len(linkinfo.llimit) > 0:
            self.minDeg = linkinfo.llimit[0]*180/math.pi
        else:
            self.minDeg = -180
        self.jointId = linkinfo.jointId
        self.name = linkinfo.name
        nameLabel = JLabel(linkinfo.name+":")
        nameLabel.setPreferredSize(Dimension(150, 15));
        self.add(nameLabel)
        self.add(JLabel("%4.0f"%self.minDeg))
        self.slider = JSlider(maximum=int(self.maxDeg), minimum=int(self.minDeg), stateChanged=self.setAngleSlider)
        self.add(self.slider)
        self.add(JLabel("%4.0f"%self.maxDeg))
        self.text = JTextField(5, actionPerformed=self.setAngleText)
        self.add(self.text)
        self.angle = angle
    def setAngleText(self, event):
        txt = self.text.getText()
        angDeg = float(txt)
        if angDeg > self.maxDeg:
            angDeg = self.maxDeg
            self.text.setText(str(angDeg))
        if angDeg < self.minDeg:
            angDeg = self.minDeg
            self.text.setText(str(angDeg))
        self.slider.setValue(int(float(angDeg)))
        self.setTargetAngle(angDeg)
    def setAngleSlider(self, event):
        angDeg = self.slider.getValue()
        self.text.setText(str(angDeg))
        self.setTargetAngle(angDeg)
    def setCurrentAngle(self,angle):
        self.slider.setValue(int(angle))
        self.angle = angle
    def setTargetAngle(self, target):
        delta = abs(self.angle - target)
        if delta == 0:
            return
        tm = delta/20.0
        if tm < 0.1:
            tm = 0.1 
        self.seq.waitInterpolation()
        self.seq.setJointAngle(self.name, target*math.pi/180, tm)
        self.angle = target
        
        
class poseEditor(JFrame):
    def __init__(self, url, seq, sh):
        ml = hrp.findModelLoader()
        self.bodyInfo = ml.getBodyInfo(url)
        self.dof = 0
        for li in self.bodyInfo.links():
            if li.jointId >= 0:
                self.dof += 1
        print "dof=",self.dof
        self.setSize(550, 800)
        self.seq = seq
        self.sh = sh
        panel = JPanel()
        panel.setLayout(BorderLayout())
        sPane = ScrollPane()
        sPane.setSize(400,720)
        jointsPanel = JPanel()
        jointsPanel.setLayout(BoxLayout(jointsPanel, 
                                        BoxLayout.Y_AXIS))
        command = CommandHolder()
        self.sh.getCommand(command)
        jav = command.value.jointRefs
        self.jpanels = []
        for li in self.bodyInfo.links():
            if li.jointId >= 0:
                jp = jointPanel(li, seq, jav[li.jointId]*180/math.pi)
                self.jpanels.append(jp)
                jointsPanel.add(jp)
        sPane.add(jointsPanel)
        panel.add(sPane, BorderLayout.NORTH)
        commandPanel = JPanel()
        commandPanel.add(JButton("get pose", actionPerformed=self.getPose))
        panel.add(commandPanel, BorderLayout.SOUTH)
        self.add(panel)
        self.getPose(None)
        self.setVisible(True)
    def getPose(self, event):
        command = CommandHolder()
        self.sh.getCommand(command)
        jav = command.value.jointRefs
        for jp in self.jpanels:
            jp.setCurrentAngle(jav[jp.jointId]*180/math.pi)



        
        
