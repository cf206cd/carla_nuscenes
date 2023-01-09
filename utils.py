import hashlib
import numpy as np
import quaternion
import json

def transform_timestamp(timestamp):
    return int(timestamp*10e6)

def get_token(key,data):
    obj = hashlib.md5(str(key).encode('utf-8'))
    obj.update(str(data).encode('utf-8'))
    result = obj.hexdigest()
    return result

def dump(data,path):
    with open(path, "w") as filedata:
        json.dump(data, filedata, indent=0, separators=(',',':'))

def load(path):
    with open(path, "r") as filedata:
        return json.load(filedata)

def get_intrinsic(fov, image_size_x,image_size_y):
    focal = image_size_x / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = image_size_x / 2.0
    K[1, 2] = image_size_y / 2.0
    return K

def get_rt(transform):
        translation = [transform.location.x,
                        transform.location.y,
                        transform.location.z]
        quat = quaternion.from_euler_angles(np.array([transform.rotation.yaw,
                                                    transform.rotation.pitch,
                                                    transform.rotation.roll
                                                    ]))
        rotation = quaternion.as_float_array(quat).tolist()
        return rotation,translation