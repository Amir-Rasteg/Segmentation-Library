Introduction

These set of scripts act as a wrapper around Open3D to add segmentation functions based on global color thresholding, and focused on medical usecases.

This system was built by Amir Rastegar as part of their Master's thesis at Duquesne University, under the supervision of Dr. Bin Yang.


### Installation
#### Step 1: Create a new Python Virtual Environment

(Skipping this step can cause a lot of pain now and/or later)

If you are using another virtual environment manager, please generate the virtual-env there and jump to step 2

##### Anaconda Instructions:

    Open Anaconda Navigator and go to Environments
    Click "Create" at the bottom
    Name it "BurnSegmentation" or whatever you wish, At the time of writing Python 3.9.x was used. Note that using a lower version may cause some issues
    Create the virtual-env and allow it to generate. Ensure you are switched to it
    Click the green play button and enter the windows terminal
    Ensure PIP is installed with conda install pip

#### Step 2: Install Dependencies

    In the terminal, navigate to the segmentation library download directory using 'cd' and 'ls' ( 'dir' for windows)
    To install all dependencies, run pip install -r requirements.txt
    Open your project in your IDE of choice in this virtual environment!
