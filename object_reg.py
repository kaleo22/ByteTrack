import pandas as pd
import numpy as np
from filterpy.kalman import KalmanFilter

class ObjectAssociator:
    def __init__(self):
        self.data_list = []
        self.Pkw = []
        self.Lkw = []
        self.Rad = []
        self.associations = {}
        self.wheel_filters = {}
        self.current_measured = set()
        self.frame_count = 0

    def init_kalman_filter(self, initial_bbox):
        """
        Initialisiert einen Kalman-Filter für ein Rad (nur x, y-Position).
        """
        kf = KalmanFilter(dim_x=4, dim_z=2)
        # Annahme konstante Geschwindigkeit deswegen dt = 1
        kf.x = np.array([initial_bbox[0], initial_bbox[1], 0, 0])
        kf.F = np.array([[1, 0, 1, 0],
                         [0, 1, 0, 1],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
        kf.H = np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0]])
        kf.P *= 1000.
        kf.R = np.eye(2) * 10
        kf.Q = np.eye(4)
        return kf

    def update_wheel_tracks(self):
        """
        Führt für jedes Rad einen Kalman-Filter-Update durch.
        """

        for _, wheels in self.associations.items():
            for idx, wheel in enumerate(wheels):
                track_id = wheel["track_id"]
                frame = wheel["frame"]
                bbox = wheel["bbox"]
                buffer = wheel["buffer"]
                label = wheel["label"]

                if track_id not in self.wheel_filters:
                    self.wheel_filters[track_id] = self.init_kalman_filter(bbox)
                kf = self.wheel_filters[track_id]

                x = (bbox[0] + bbox[2]) / 2
                y = (bbox[1] + bbox[3]) / 2
                kf.predict()
                kf.update([x, y])

                w = bbox[2] - bbox[0]
                h = bbox[3] - bbox[1]
                
                x1 = kf.x[0] - w / 2
                y1 = kf.x[1] - h / 2
                x2 = kf.x[0] + w / 2
                y2 = kf.x[1] + h / 2

                wheel_est = {
                    "frame": frame,
                    "bbox": [x1, y1, x2, y2],
                    "track_id": track_id,
                    "label": label,
                    "buffer": buffer + 1
                }
                #idx = next((i for i, w in enumerate(wheels) if w["track_id"] == track_id), None)
                wheels[idx] = wheel_est

                if wheel_est["buffer"] >= 15 and track_id in self.wheel_filters:
                    del self.wheel_filters[track_id]

        #return wheel_est

    def data_collect(self, wheel_id, vehicle_id, iob_value, vehicle_type):
        data = {
            "wheel_id": wheel_id,
            "vehicle_id": vehicle_id,
            "iob_value": iob_value,
            "frame": self.frame_count,
            "vehicle_type": vehicle_type
        }
        self.data_list.append(data)

    def to_dataframe(self):
        return pd.DataFrame(self.data_list)

    def register_objects(self, bbox, target):
 
        self.Pkw = []
        self.Lkw = []
        self.Rad = []

        if target.label == "car" and target.track_id not in [t[2] for t in self.Pkw]:
            self.Pkw.append((self.frame_count, bbox, target.track_id, target.label))
        elif target.label == "truck" and target.track_id not in [t[2] for t in self.Lkw]:
            self.Lkw.append((self.frame_count, bbox, target.track_id, target.label))
        elif target.label == "wheel" and target.track_id not in [t[2] for t in self.Rad]:
            self.Rad.append((self.frame_count, bbox, target.track_id, target.label))


    def associate_wheels_to_vehicles(self):
        frame = self.frame_count
        current_measured = set()

        for wheel in self.Rad:
            best_iob = 0
            best_vehicle = None
            for vehicle in self.Pkw + self.Lkw:
                iob = bbox_intersection(wheel[1], vehicle[1])
                self.data_collect(wheel[2], vehicle[2], iob, vehicle[3])
                if iob > best_iob:
                    best_iob = iob
                    best_vehicle = vehicle
            if best_iob > 0.5 and best_vehicle is not None:
                v_id = best_vehicle[2]

                if v_id not in self.associations:
                    self.associations[v_id] = []

                existing_idx = next((i for i, w in enumerate(self.associations[v_id]) if w["track_id"] == wheel[2]), None)
                wheel_dict = {
                    "frame": wheel[0],
                    "bbox": wheel[1],
                    "track_id": wheel[2],
                    "label": wheel[3],
                    "buffer": 0
                }

                if existing_idx is None:
                    self.associations[v_id].append(wheel_dict)
                else:
                    self.associations[v_id][existing_idx] = wheel_dict
                current_measured.add(wheel[2])
        self.current_measured = current_measured

        return self.associations

    def classify_objects(self):
        """
        Gibt ein Dictionary mit vehicle_id als Schlüssel und der Klasse als Wert zurück.
        """
        classification = {}
        for v_id, wheels in self.associations.items():
            if not wheels:
                continue
            label = wheels[0]["label"]
            num_wheels = len(wheels)
            if label == "car":
                classification[v_id] = "Auto"
            elif label == "truck":
                if num_wheels == 2:
                    classification[v_id] = "Einachser"
                elif num_wheels == 3:
                    classification[v_id] = "Doppelachser"
                elif num_wheels > 3:
                    classification[v_id] = "Dreifachachser"
                else:
                    classification[v_id] = "Unbekannt"
            else:
                classification[v_id] = "Unbekannt"
        return classification

def bbox_intersection(wheelbox, vehiclebox):
            # wheelbox und vehiclebox: [x1, y1, x2, y2]
            xA = max(wheelbox[0], vehiclebox[0])
            yA = max(wheelbox[1], vehiclebox[1])
            xB = min(wheelbox[2], vehiclebox[2])
            yB = min(wheelbox[3], vehiclebox[3])

            interArea = max(0, xB - xA) * max(0, yB - yA)
            if interArea == 0:
                return 0.0

            wheelboxArea = (wheelbox[2] - wheelbox[0]) * (wheelbox[3] - wheelbox[1])
            #vehicleboxArea = (vehiclebox[2] - vehiclebox[0]) * (vehiclebox[3] - vehiclebox[1])
            intersection = interArea / float(wheelboxArea)
            return intersection
