import os
import xml.etree.ElementTree as ET

models_root_path = "/home/marvin/ros2_ws/src/orbslam_nav_sim/models"

models_folder_list = os.listdir(models_root_path)

for models_folder in models_folder_list:
    sdf_path = os.path.join(models_root_path, models_folder, "model.sdf")
    
    # Load the SDF file
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    # Iterate through all <uri> tags and replace
    for uri in root.iter('uri'):
        if uri.text.startswith('file://'):
            # Replace only the start
            uri.text = uri.text.replace('file://', 'model://', 1)
        uri.text = uri.text.replace('models/', '', 1)

    # Save back to file (or to a new file)
    tree.write(sdf_path, encoding='utf-8', xml_declaration=True)
