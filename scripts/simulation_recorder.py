from std_msgs.msg import String
import rospy
from ambf_client import Client
import time
import os
import math
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tkinter import Tk, Label, Button, PhotoImage, Scale, HORIZONTAL, Entry, Spinbox
from tkinter import ttk
from PIL import ImageTk, Image


class recording_class:

    def __init__(self):
        # Initialize arrays to store position and Rpy
        self.drillPos_list = []
        self.drillRpy_list = []
        self.drillPos_arr = []
        self.drillRpy_arr = []

        # Initialize recording and playback flags and variables
        self.recording = 0
        self.recording_flag = False
        self.forward = True
        self.fast = False
        self.playing = False
        self.step = 0
        self.captureRate = 10 # Should be 10 # 10 fps is the default value  every 10 ms)
        self.currentIndex = 0
        self.buttonUpdated = 0

        # Initialize global widget variables
        self.root = Tk()
        self.sliderDefined = False
        self.speedComboBox = None
        self.stepComboBox = None
        self.nameEntry = None
        self.dateEntry = None
        self.speedComboBox = None
        self.sliderWidget = None
        self.sliderLabel= None
        self.sliderDefined = 0
        self.moveToLabel = None
        self.posEntry = None
        self.step = 0
        self.captureRate= 10
        self.currentIndex = 0
        self.recordButton = None

        # Set up the python client connection
        bridge = CvBridge()
        self._client = Client()
        #self._client.connect()

        # Define object name
        self.mDrill_name = '/ambf/env/mastoidectomy_drill'

    def initialize_widget(self):
        """
        This function initializes the record and playback widget, connects all buttons
        to their respective functions and launches functionality
        """
        path = os.getcwd()
        # Rewind button
        rewindButtonImage = self.createButtonImage(path + "/Icons/RW.png")
        self.createButton(rewindButtonImage, self.rewind)

        # Previous button
        prevImage = self.createButtonImage(path + "/Icons/Prev.png")
        self.createButton(prevImage, self.moveToPreviousPos)

        # Record button
        recordButtonImage = self.createButtonImage(path + "/Icons/Record.png")
        self.createButton(recordButtonImage, self.recordButtonClicked)

        # Play button
        playButtonImage = self.createButtonImage(path + "/Icons/Play.png")
        self.createButton(playButtonImage, self.play)#playbackRecording)
        
        # Pause button
        pauseButtonImage = self.createButtonImage(path + "/Icons/Pause.png")
        self.createButton(pauseButtonImage, self.pause)

        # Next button
        nextImage = self.createButtonImage(path + "/Icons/Next.png")
        self.createButton(nextImage, self.moveToNextPos)
    
        # Fast forward button
        ffButtonImage = self.createButtonImage(path + "/Icons/FF.png")
        self.createButton(ffButtonImage, self.fastForward)

        # Additional labels/ slider widgets
        label = Label(self.root, text='Capture rate (fps):')
        label.pack(side='left')
        self.speedComboBox = ttk.Combobox(self.root, values=["100", "200", "300"], width=3)
        self.speedComboBox.pack(side='left')
        self.speedComboBox.set('100')
        self.stepComboBox = ttk.Combobox(self.root, values=["Frames", "Seconds"], width=7)
        self.stepComboBox.pack(side='left')
        self.stepComboBox.bind("<<ComboboxSelected>>", self.updateStepValue)
        self.stepComboBox.set("Seconds")
        label = Label(self.root, text='Name')
        label.pack(side='left')
        self.nameEntry = Entry(self.root, bd=5)
        self.nameEntry.pack(side='left')
        label = Label(self.root, text='Date')
        label.pack(side='left')
        self.dateEntry = Entry(self.root, bd=3)
        self.dateEntry.pack(side='left')
        saveImage = self.createButtonImage(path + "/Icons/Save.png")
        self.createButton(saveImage, self.save)

        # Start main loop
        self.root.after(self.captureRate, self.recordDrillPos) #10 fps (100 ms)
        self.root.mainloop()
    
    def recordDrillPos(self):
        """
        This function records the drill position  when the record button is pressed
        and intializes different playback modes when the play button is pressed
        """

        selectedSpeed = self.updateSpeed()

        # Enter this statement if recording is active
        if self.recording_flag:

            # If this is the second time pressing record refresh the data storage to zero
            if self.drillPos_arr is not None:
                self.drillPos_arr = None
                self.drillRpy_arr = None
                self.drillPos_list = []
                self.drillRpy_list = []
            
            # Get the object handle
            mDrill_obj = self._client.get_obj_handle(self.mDrill_name)

            # Get and store the drill Pos
            drill_Pos = mDrill_obj.get_pos()
            self.drillPos_list.append([drill_Pos.x, drill_Pos.y, drill_Pos.z])

            # Get and store the drill Rpy 
            drill_Rpy = mDrill_obj.get_rpy()
            self.drillRpy_list.append([drill_Rpy[0], drill_Rpy[1], drill_Rpy[2]])

            time.sleep(selectedSpeed)

        # Enter this loop if playback is active
        if self.playing:

            # Play at standard speed
            if self.forward is True:
                self.playSliderPos(self.currentIndex)
                self.sliderWidget.set(self.currentIndex)
                self.sliderWidget.pack()
                self.root.update()

                if self.currentIndex == len(self.drillPos_arr): # end of the recording
                    self.currentIndex = 0 # Reset to the beginning
                self.currentIndex = self.currentIndex + 1 # iterate the counter
                
                # Fast forward settings
                if self.fast is True:
                    time.sleep(selectedSpeed/5)
                else:
                    time.sleep(selectedSpeed)
            
            # Rewind recording
            if self.forward is False:
                if self.currentIndex == 0: # Beginning of the recording
                    self.currentIndex = len(self.drillPos_arr) # Reset to the end

                self.playSliderPos(self.currentIndex)
                self.sliderWidget.set(self.currentIndex)
                self.sliderWidget.pack()
                self.root.update()
                self.currentIndex = self.currentIndex - 1
                time.sleep(selectedSpeed/5)

        self.root.after(100, self.recordDrillPos) # should be 100


    def createButtonImage(self, imagePath):
        """
        This function converts the saved image into the proper format for the widget
        """
        image = Image.open(imagePath)
        image_resized = image.resize((30, 30), Image.ANTIALIAS)
        photo_resized = ImageTk.PhotoImage(image_resized)
        return photo_resized

    def createButton(self, imagePath, commandName):
        """
        This function initializes a button with an image and a callback function on
        the UI
        """
        if 'recordButtonClicked' in str(commandName):
            self.recordButton = Button(self.root, image=imagePath, command=commandName)
            self.recordButton.pack(side='left')
        else:
            button = Button(self.root, image=imagePath, command=commandName)
            button.pack(side='left')

    def rewind(self):
        """
        This function replays the full recording from end to beginning (in fast mode)
        """
        self.forward = False
        self.fast = True
        self.playing = True
    
    def updateSpeed(self):
        """
        This function takes the set speed from the UI and updates the selected speed accordinly
        """
        selectedSpeed = self.speedComboBox.get()
        selectedSpeed = float(selectedSpeed)
        selectedSpeed = selectedSpeed/1000
        return selectedSpeed
    
    def moveToPreviousPos(self):
        """
        This function enables stepping backward in the recording
        """
        if self.sliderWidget is not None:
            currentPos = self.sliderWidget.get()
            if currentPos != 0: # indicating it's at the end of the recording
                prevPos = currentPos - 1
                self.playSliderPos(prevPos)
                self.sliderWidget.set(prevPos)
                self.sliderWidget.pack()
                self.root.update()
                self.currentIndex = prevPos
    
    def recordButtonClicked(self):
        """
        This function is used to toggle the record button state
        """
        global recording
        if self.recording == 0:
            self.recording = 1
            self.startRecording()
        else:
            self.recording = 0
            self.stopRecording()

    def startRecording(self):
        """
        This function initiates the position recording
        """
        self.recording_flag = True
        self.updateButtonImage()
        #self.resetDrillPos() #TODO: ask how to release the drill pos from the object handle to fix error on multiple recording
        print('Recording started.')
        # Start with clean up first (in case the control mode is switched)
        self._client.clean_up()
        self._client.connect()


    def stopRecording(self):
        """
        This function stops the position recording and converts the Pos and Rpy lists int       o np arrays
        """
        self.recording_flag = False
        self.updateButtonImage()
        print('Recording stopped.')

        # Convert the lists into numpy arrays
        self.drillPos_arr = np.asarray(self.drillPos_list)
        self.drillRpy_arr = np.asarray(self.drillRpy_list)
        self.updateSliderWidget()

    def updateButtonImage(self):
        """
        This function updates the button image on the record button.
        :return:
        """
        path = os.getcwd()
        if self.recording_flag == True:
            stopButtonImage = self.createButtonImage(path + "/Icons/Stop.png")
            self.recordButton.configure(image=stopButtonImage)
            self.recordButton.image = stopButtonImage
            self.root.update()
        elif self.recording_flag == False:
            recordButtonImage = self.createButtonImage(path + "/Icons/Record.png")
            self.recordButton.configure(image=recordButtonImage)
            self.recordButton.image = recordButtonImage
            self.root.update()

    def updateSliderWidget(self):
        """
        This function updates the UI to show the slider widget if there is a recording finished.
        """
        # Check if there is already a sliderWidget in the UI
        if self.sliderDefined is True:
            # Avoid multiple slider widgets
            if self.sliderWidget is not None:
                self.sliderWidget.destroy()
            if self.sliderLabel is not None:
                self.sliderLabel.destroy()
            if self.posEntry is not None:
                self.posEntry.destroy()
            if self.moveToLabel is not None:
                self.moveToLabel.destroy()
            self.sliderDefined = False

        # Define the range of the slider widget based on the length of the recorded data
        if self.drillRpy_arr is not None:
            self.sliderWidget = Scale(self.root, from_=0, to=len(self.drillRpy_arr), showvalue=False, orient=HORIZONTAL, command=self.playSliderPos)
            self.sliderDefined = True
            self.sliderLabel = Label(self.root, text="")
            self.sliderWidget.pack()
            self.sliderLabel.pack(side='bottom')

    def play(self):
        """
        This function initializes playback by setting playback flags
        """
        if self.recording_flag:
            print('Cannot play while recording')
            return
        
        self.playing = True
        self.fast = False
        self.forward = True
        print('Playback started')

    def pause(self):
        """
        This function pauses whatever is currently set to run in the main loop.
        """
        if self.recording_flag:
            return
        self.playing = False
        print('Playback is paused.')

    def fastForward(self):
        """
        This function replays the full recording from beginning to end quickly
        """
        self.forward = True
        self.fast = True
        self.playing = True

    def rewind(self):
        """
        This function replays the full recording from end to beginning quickly
        """ 
        self.forward = False
        self.fast = True
        self.playing = True
    
    def moveToNextPos(self):
        """
        This function enables stepping forward in the recording
        """
        if self.sliderWidget is not None:
            currentPos = self.sliderWidget.get()
            if currentPos != len(self.drillPos_arr): # indicating it's at the end of the recording
                nextPos = currentPos + 1
                self.playSliderPos(nextPos)
                self.sliderWidget.set(nextPos)
                self.sliderWidget.pack()
                self.root.update()
                self.currentIndex = nextPos
   

    def playSliderPos(self,index):
        """
        This function puts the drill at the position indicated by the slider widget
        """

        pos = int(float(index)) - 1 # Convert the current slider position to the correct index

        mDrill_obj = self._client.get_obj_handle(self.mDrill_name)
        mDrill_obj.set_pos(self.drillPos_arr[pos, 0], self.drillPos_arr[pos, 1], self.drillPos_arr[pos, 2])
        mDrill_obj.set_rpy(self.drillRpy_arr[pos, 0], self.drillRpy_arr[pos,1], self.drillRpy_arr[pos, 2])

        if self.step == 0:
            # Update the label in seconds
            milliseconds = pos%100 # each pos is actually 10 ms but the display accounts for this
            seconds = pos/100
            self.sliderLabel.configure(text="%2.2d:%2.2d" % (seconds, milliseconds))
        else:
            # Update the label in frame
            self.sliderLabel.configure(text="Frame {}/{}".format(pos, len(self.drillPos_arr)))

    def resetDrillPos(self):
        #This function should release the drill from the set pos and rpy - ask how to do this
        mDrill_obj = self._client.get_obj_handle(self.mDrill_name)
        drill_Pos = mDrill_obj.get_pos()
        mDrill_obj.set_pos(drill_Pos.x, drill_Pos.y, drill_Pos.z)
        drill_Rpy = mDrill_obj.get_rpy()
        mDrill_obj.set_rpy(drill_Rpy[0], drill_Rpy[1], drill_Rpy[2])

    def updateStepValue(self,selectedStep):
        """
        This function updates the step display for the recording (toggle between
        frames and seconds)
        """
        selectedStep = self.stepComboBox.get()

        if selectedStep == 'Frames':
            self.activateFrameSteps()
        else:
            self.activateSecondSteps()

        currentPos = self.sliderWidget.get()
        self.playSliderPos(currentPos)
        self.root.update()

    def activateFrameSteps(self):
        """
        This function sets display to show step size as frames
        """
        self.step = 1

    def activateSecondSteps(self):
        """
        This function sets display to show step size as seconds
        """
        self.step = 0

    def save(self):
        """
        This function allows the user to save the recording in .npz format according
        to the name and date entered in the UI.
        """
        if len(self.drillPos_arr) == 0 or self.drillPos_arr is None:
            print('Saving incomplete. Begin recording first.')
            return
        path = os.getcwd()
        np.savez(path + '/Recorded_Sessions/' + self.nameEntry.get() + '_' + self.dateEntry.get() + '.npz', name1=self.drillPos_arr, name2=self.drillRpy_arr)


p = recording_class()
p.initialize_widget()

