#see managed and passive viewer documentation here
# https://mujoco.readthedocs.io/en/stable/python.html
import mujoco
import mujoco.viewer
import os

xml_path = "hello.xml"
dirname = os.path.dirname(__file__)
abs_path = os.path.join(dirname + "/" + xml_path)

model_name = 'model.txt'
model_path = os.path.join(dirname + "/" + model_name)

# Load the model and create simulation data
model = mujoco.MjModel.from_xml_path(abs_path)

#print the model
mujoco.mj_printModel(model,model_path)

#Load the data
data = mujoco.MjData(model)

# Launch the MuJoCo viewer
mujoco.viewer.launch(model, data)
