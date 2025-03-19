!pip install roboflow

from roboflow import Roboflow
rf = Roboflow(api_key="kyF9BPXgpDNmng0FiQkv")
project = rf.workspace("liverpoolrc").project("buckets-v2")
version = project.version(1)
dataset = version.download("yolov8")
                
