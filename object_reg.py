import pandas as pd
import numpy as np
from filterpy.kalman import KalmanFilter
import cv2

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
 
        # self.Pkw = []
        # self.Lkw = []
        # self.Rad = []
        # Braucht noch eine Update Funktion für current_rad, _pkw, __lkw
        if target.label == "car":
            idx = next((i for i, t in enumerate(self.Pkw) if t[2] == target.track_id), None)
            if idx is None:
                self.Pkw.append((self.frame_count, bbox, target.track_id, target.label))
            else:
                 self.Pkw[idx] = (self.frame_count, bbox, target.track_id, target.label)
        elif target.label == "truck":
            idx = next((i for i, t in enumerate(self.Lkw) if t[2] == target.track_id), None)
            if idx is None:
                self.Lkw.append((self.frame_count, bbox, target.track_id, target.label))
            else:
                self.Lkw[idx] = (self.frame_count, bbox, target.track_id, target.label)
        elif target.label == "wheel":
            idx = next((i for i, t in enumerate(self.Rad) if t[2] == target.track_id), None)
            if idx is None:
                self.Rad.append((self.frame_count, bbox, target.track_id, target.label))
            else:
                self.Rad[idx] = (self.frame_count, bbox, target.track_id, target.label)

    def associate_wheels_to_vehicles(self):
        frame = self.frame_count
        current_measured = set()

        current_rad = [wheel for wheel in self.Rad if wheel[0] == frame]
        current_pkw = [car for car in self.Pkw if car[0] == frame]
        current_lkw = [truck for truck in self.Lkw if truck[0] == frame]

        for wheel in current_rad:
            best_iob = 0
            best_vehicle = None
            for vehicle in current_pkw + current_lkw:
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
            vehicle = next((v for v in self.Pkw + self.Lkw if v[2] == v_id), None)
            if vehicle is not None:
                label = vehicle[3]
            else:
                label = None
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
    
    def _get_vehicle_entry(self, v_id):
        """
        Liefert den letzten Eintrag (frame, bbox, track_id, label) für vehicle id.
        """
        candidates = [v for v in (self.Pkw + self.Lkw) if v[2] == v_id]
        if not candidates:
            return None
        return max(candidates, key=lambda t: t[0])  # größtes frame = zuletzt gesehen

    @staticmethod
    def _center(bbox):
        x1, y1, x2, y2 = bbox
        return (0.5*(x1+x2), 0.5*(y1+y2))

    def ax_distance_bb(self, classification):
        """
        classification: dict {vehicle_id: class_string}
        Returns:
            dict {vehicle_id: {"axle_x": [...], "axle_dist": [...], "vehicle_bbox": [...], "wheel_centers": [...]} }
        """
        out = {}

        for v_id, cls in classification.items():
            wheels = self.associations.get(v_id, [])
            veh = self._get_vehicle_entry(v_id)

            if veh is None or not wheels:
                out[v_id] = {"axle_x": [], "axle_dist": [], "vehicle_bbox": None, "wheel_centers": []}
                continue

            vehicle_bbox = veh[1]

            # 1) Wheel centers holen
            wheel_centers = [self._center(w["bbox"]) for w in wheels]
            wheel_x = np.array([c[0] for c in wheel_centers], dtype=float)

            # 2) Wenn du schon Achsen gruppiert hast -> HIER deine Achsenlogik rein
            # Minimal: unique/cluster entlang x (du ersetzt das durch dein fertiges Clustering)
            # Beispiel: sortieren und "nahe" Werte zusammenfassen
            wheel_x_sorted = np.sort(wheel_x)

            # --- Replace-Block: deine fertige Achsen-Gruppierung ---
            # Hier ein ultra-minimaler Platzhalter:
            # threshold z.B. 0.08 der Fahrzeugbreite im Bild
            vx1, vy1, vx2, vy2 = vehicle_bbox
            vW = max(1.0, (vx2 - vx1))
            thr = 0.08 * vW

            axle_x = []
            current = [wheel_x_sorted[0]]
            for x in wheel_x_sorted[1:]:
                if abs(x - current[-1]) <= thr:
                    current.append(x)
                else:
                    axle_x.append(float(np.mean(current)))
                    current = [x]
            axle_x.append(float(np.mean(current)))
            axle_x.sort()
            # --- /Replace-Block ---

            axle_dist = [abs(axle_x[i+1] - axle_x[i]) for i in range(len(axle_x)-1)]

            out[v_id] = {
                "axle_x": axle_x,
                "axle_dist": axle_dist,
                "vehicle_bbox": vehicle_bbox,
                "wheel_centers": wheel_centers
            }

        return out 

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


