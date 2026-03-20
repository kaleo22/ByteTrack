import math

class DistanceTracker:  
    def __init__(self):
        self.rad_measures = {}  # Speichert:   {(ID1, ID2): max_normalisierte_distanz}
        self.frame_objects = {}  # Temporär für aktuellen Frame:   {frame:   [(target, bbox), ...]}
    
    def register_objects(self, bbox, target, frame):
        """Sammelt Objekte pro Frame"""
        if frame not in self.frame_objects:
            self.frame_objects[frame] = []
        
        self.frame_objects[frame].append((target, bbox))
    
    def berechne_bbox_flaeche(self, bbox):
        """Berechnet Fläche der bbox [x1, y1, x2, y2]"""
        x1, y1, x2, y2 = bbox
        return (x2 - x1) * (y2 - y1)
    
    def berechne_zentrum_distanz(self, bbox1, bbox2):
        """Berechnet euklidische Distanz zwischen bbox-Zentren"""
        x1, y1, x2, y2 = bbox1
        zentrum1_x = (x1 + x2) / 2
        zentrum1_y = (y1 + y2) / 2
        
        x1, y1, x2, y2 = bbox2
        zentrum2_x = (x1 + x2) / 2
        zentrum2_y = (y1 + y2) / 2
        
        distanz = math.sqrt((zentrum2_x - zentrum1_x)**2 + (zentrum2_y - zentrum1_y)**2)
        return distanz
    
    def verarbeite_frame(self, frame):
        """Verarbeitet alle Objekte eines Frames und berechnet Distanzen zwischen benachbarten IDs"""
        if frame not in self.frame_objects:
            return
        
        # Sortiere nach track_id (numerisch)
        objekte = sorted(self.frame_objects[frame], key=lambda x: x[0]. track_id)
        
        # Iteriere über benachbarte Paare (ID1↔ID2, ID2↔ID3, etc.)
        for i in range(len(objekte) - 1):
            target1, bbox1 = objekte[i]
            target2, bbox2 = objekte[i + 1]
            
            ID1 = target1.track_id
            ID2 = target2.track_id
            
            # Berechne Distanz zwischen Zentren
            distanz = self.berechne_zentrum_distanz(bbox1, bbox2)
            
            # Berechne kombinierte Fläche
            flaeche1 = self.berechne_bbox_flaeche(bbox1)
            flaeche2 = self.berechne_bbox_flaeche(bbox2)
            kombinierte_flaeche = (flaeche1 + flaeche2)/2   
            
            # Normalisierte Distanz
            if kombinierte_flaeche > 0:
                norm_distanz = distanz / kombinierte_flaeche
            else:
                norm_distanz = 0
            
            # Speichere nur wenn größer als bisheriger Wert
            id_paar = (ID1, ID2)
            if id_paar not in self.rad_measures or norm_distanz > self.rad_measures[id_paar]:  
                self.rad_measures[id_paar] = norm_distanz
        
        # Optional: Lösche Frame-Daten nach Verarbeitung um Speicher zu sparen
        # del self.frame_objects[frame]
    
    def hole_distanz(self, ID1, ID2):
        """Holt gespeicherte maximale normalisierte Distanz für ein ID-Paar"""
        return self.rad_measures.get((ID1, ID2), None)
    
    def hole_alle_distanzen(self):
        """Gibt alle gespeicherten ID-Paare und ihre Distanzen zurück"""
        return self.rad_measures
    
    def clear_frame(self, frame):
        """Löscht Frame-Daten um Speicher freizugeben"""
        if frame in self.frame_objects:
            del self.frame_objects[frame]


def bbox_intersection(wheelbox, vehiclebox):
    """
    Berechnet Intersection over Box (IoB) - wie viel % der Wheel-Box überlappen mit der Vehicle-Box. 
    """
    # wheelbox und vehiclebox: [x1, y1, x2, y2]
    xA = max(wheelbox[0], vehiclebox[0])
    yA = max(wheelbox[1], vehiclebox[1])
    xB = min(wheelbox[2], vehiclebox[2])
    yB = min(wheelbox[3], vehiclebox[3])

    interArea = max(0, xB - xA) * max(0, yB - yA)
    if interArea == 0:
        return 0.0

    wheelboxArea = (wheelbox[2] - wheelbox[0]) * (wheelbox[3] - wheelbox[1])
    intersection = interArea / float(wheelboxArea)
    print(f"IOB Berechnet: {intersection:.4f}")
    return intersection


class Associator:
    
    def __init__(self, iob_schwellwert=0.5):
        """
        Args:
            iob_schwellwert: Intersection over Box Schwellwert für Rad-Fahrzeug-Assoziierung
        """
        self. Pkw = []
        self.Lkw = []
        self. Rad = []
        self.Traktor = []
        self.Bus = []
        self.Transporter = []
        self.Anhaenger = []
        self.associations = {}
        self.frame_count = 0
        self.current_measured = set()
        self.iob_schwellwert = iob_schwellwert
        
        # NEU: DistanceTracker hinzufügen
        self.distance_tracker = DistanceTracker()

    def print_debug_info(self):
        """Gibt Debug-Informationen aus"""
        print(f"\n{'='*60}")
        print(f"DEBUG INFO - Frame {self.frame_count}")
        print(f"{'='*60}")
        print(f"Anzahl registrierter Räder (gesamt): {len(self.Rad)}")
        print(f"Anzahl registrierter PKW (gesamt): {len(self.Pkw)}")
        print(f"Anzahl registrierter LKW (gesamt): {len(self.Lkw)}")
        print(f"Anzahl registrierter Busse (gesamt): {len(self.Bus)}")
        
        # Zeige unique Rad-IDs
        rad_ids = list(set([r[2] for r in self.Rad]))
        print(f"Unique Rad Track-IDs: {sorted(rad_ids)}")
        
        # Zeige unique Fahrzeug-IDs
        vehicle_ids = list(set([v[2] for v in self. Pkw + self.Lkw]))
        print(f"Unique Fahrzeug Track-IDs:  {sorted(vehicle_ids)}")
        
        # Zeige Assoziierungen
        print(f"\nAssoziierungen:")
        for v_id, wheels in self.associations.items():
            wheel_ids = [w['track_id'] for w in wheels]
            iobs = [w['iob'] for w in wheels]
            print(f"  Fahrzeug {v_id}:  {len(wheels)} Räder - IDs: {wheel_ids}")
            print(f"    IoB-Werte: {[f'{iob:.3f}' for iob in iobs]}")
        
        # Zeige Rad-Distanzen
        distanzen = self.distance_tracker.hole_alle_distanzen()
        if distanzen:
            print(f"\nGemessene Rad-Distanzen:")
            for (id1, id2), dist in sorted(distanzen.items()):
                print(f"  Rad {id1} ↔ Rad {id2}: {dist:.4f}")
    
    def register_objects(self, bbox, target):
        """Registriert Objekte und speichert sie"""
        if target. label == "car":
            idx = next((i for i, t in enumerate(self.Pkw) if t[2] == target.track_id), None)
            if idx is None:
                self.Pkw.append((self.frame_count, bbox, target. track_id, target.label))
            else:
                self.Pkw[idx] = (self.frame_count, bbox, target.track_id, target.label)
        elif target.label == "truck":
            idx = next((i for i, t in enumerate(self.Lkw) if t[2] == target.track_id), None)
            if idx is None:
                self. Lkw.append((self. frame_count, bbox, target. track_id, target.label))
            else:
                self. Lkw[idx] = (self.frame_count, bbox, target.track_id, target.label)
        elif target.label == "wheel":
            idx = next((i for i, t in enumerate(self.Rad) if t[2] == target.track_id), None)
            if idx is None:
                self.Rad. append((self.frame_count, bbox, target.track_id, target.label))
            else:
                self.Rad[idx] = (self.frame_count, bbox, target.track_id, target.label)
        elif target.label == "tractor":
            idx = next((i for i, t in enumerate(self.Traktor) if t[2] == target.track_id), None)
            if idx is None:
                self.Traktor.append((self.frame_count, bbox, target.track_id, target.label))
            else:
                self.Traktor[idx] = (self.frame_count, bbox, target.track_id, target.label)
        elif target.label == "bus":
            idx = next((i for i, t in enumerate(self.Bus) if t[2] == target.track_id), None)
            if idx is None:
                self.Bus.append((self.frame_count, bbox, target.track_id, target.label))
            else:
                self.Bus[idx] = (self.frame_count, bbox, target.track_id, target.label)
        elif target.label == "van":
            idx = next((i for i, t in enumerate(self.Transporter) if t[2] == target.track_id), None)
            if idx is None:
                self.Transporter.append((self.frame_count, bbox, target.track_id, target.label))
            else:
                self.Transporter[idx] = (self.frame_count, bbox, target.track_id, target.label)
            
            # Registriere Räder auch im DistanceTracker
            self. distance_tracker.register_objects(bbox, target, self.frame_count)
    
    def associate_wheels_to_vehicles(self):
        """Assoziiert Räder zu Fahrzeugen basierend auf IoB-Schwellwert"""
        frame = self.frame_count
        current_measured = set()

        current_rad = [wheel for wheel in self.Rad if wheel[0] == frame]
        current_pkw = [car for car in self.Pkw if car[0] == frame]
        current_lkw = [truck for truck in self. Lkw if truck[0] == frame]
        current_traktor = [tractor for tractor in self.Traktor if tractor[0] == frame]
        current_bus = [bus for bus in self.Bus if bus[0] == frame]
        current_transporter = [van for van in self.Transporter if van[0] == frame]
        current_trailer = [trailer for trailer in self.Anhaenger if trailer[0] == frame]

        for wheel in current_rad:
            best_iob = 0
            best_vehicle = None
            
            for vehicle in current_pkw + current_lkw + current_traktor + current_bus + current_transporter + current_trailer:
                iob = bbox_intersection(wheel[1], vehicle[1])
                
                if iob > best_iob:
                    best_iob = iob
                    best_vehicle = vehicle
            
            # Nur assoziieren wenn IoB-Schwellwert überschritten wird
            if best_iob > self.iob_schwellwert and best_vehicle is not None:
                v_id = best_vehicle[2]

                if v_id not in self.associations:
                    self. associations[v_id] = []

                existing_idx = next((i for i, w in enumerate(self.associations[v_id]) if w["track_id"] == wheel[2]), None)
                wheel_dict = {
                    "frame":  wheel[0],
                    "bbox": wheel[1],
                    "track_id": wheel[2],
                    "label": wheel[3],
                    "buffer": 0,
                    "iob": best_iob  # Speichere IoB-Wert
                }

                if existing_idx is None: 
                    self.associations[v_id].append(wheel_dict)
                else:
                    self.associations[v_id][existing_idx] = wheel_dict
                current_measured.add(wheel[2])
        
        self.current_measured = current_measured
        
        # Verarbeite Frame für Distanzberechnung
        self.distance_tracker.verarbeite_frame(frame)

        return self.associations
    
    def analysiere_achskonfiguration(self, wheel_ids, distanz_schwellwert=0.07):
        """
        Analysiert die Achskonfiguration basierend auf Abständen zwischen benachbarten Rädern. 
        Jedes Rad = eine Achse! 
        
        Args:
            wheel_ids: Liste von Rad track_ids
            distanz_schwellwert: Normalisierte Distanz ab der Achsen in verschiedene Gruppen fallen
            
        Returns:
            Dictionary mit:  
            - "achsen": Liste von Rad-IDs (jedes Rad = 1 Achse)
            - "achsgruppen": [[achse1, achse2], [achse3]] - Gruppierung nach vorne/hinten
            - "konfiguration": "1x3" oder "2x2" etc. 
            - "distanzen": [(id1, id2, distanz), ...] - Alle gemessenen Distanzen
        """
        if len(wheel_ids) < 1:
            return {
                "achsen": wheel_ids,
                "achsgruppen": [wheel_ids] if wheel_ids else [],
                "konfiguration": f"{len(wheel_ids)}x0" if len(wheel_ids) == 1 else "0x0",
                "distanzen": [],
                "anzahl_achsen": len(wheel_ids),
                "anzahl_achsgruppen": 1 if wheel_ids else 0
            }
        
        # Sortiere wheel_ids - jedes Rad ist eine Achse
        sorted_ids = sorted(wheel_ids)
        achsen = sorted_ids  # Jedes Rad = 1 Achse
        
        if len(sorted_ids) < 2:
            return {
                "achsen": achsen,
                "achsgruppen": [achsen],
                "konfiguration": f"{len(achsen)}",
                "distanzen": [],
                "anzahl_achsen": len(achsen),
                "anzahl_achsgruppen": 1
            }
        
        # Sammle alle Distanzen zwischen benachbarten Achsen (Rädern)
        distanzen = []
        for i in range(len(sorted_ids) - 1):
            id1 = sorted_ids[i]
            id2 = sorted_ids[i + 1]
            distanz = self.distance_tracker.hole_distanz(id1, id2)
            distanzen. append((id1, id2, distanz if distanz is not None else 0))
        
        # Gruppiere Achsen in Achsgruppen (vorne/hinten)
        # Große Distanz = neue Achsgruppe
        achsgruppen = []
        aktuelle_gruppe = [sorted_ids[0]]
        
        for i, (id1, id2, distanz) in enumerate(distanzen):
            if distanz < distanz_schwellwert: 
                # Kleine Distanz -> gleiche Achsgruppe (z.B.  Tandem/Tridem)
                aktuelle_gruppe.append(id2)
            else:
                # Große Distanz -> neue Achsgruppe (vorne vs.  hinten)
                achsgruppen.append(aktuelle_gruppe)
                aktuelle_gruppe = [id2]
        
        # Letzte Gruppe hinzufügen
        achsgruppen.append(aktuelle_gruppe)
        
        # Erstelle Konfigurationsstring (z.B. "1x3" oder "2x2")
        gruppen_groessen = [len(gruppe) for gruppe in achsgruppen]
        konfiguration = "x".join(map(str, gruppen_groessen))
        
        return {
            "achsen": achsen,
            "achsgruppen": achsgruppen,
            "konfiguration": konfiguration,
            "distanzen": distanzen,
            "anzahl_achsen": len(achsen),
            "anzahl_achsgruppen": len(achsgruppen)
        }
    
    def klassifiziere_lkw_typ(self, konfiguration, anzahl_achsen):
        """
        Klassifiziert LKW-Typ basierend auf Achskonfiguration.
        
        Args:
            konfiguration: String wie "1x1", "1x2", "2x2", "1x3" etc.
            anzahl_achsen: Gesamtanzahl der Achsen
            
        Returns:
            String mit LKW-Typ
        """
        # Mapping von Konfiguration zu Typ
        typ_mapping = {
            "1x1": "Zweiachser (1+1)",
            "2x1": "Zweiachser (2+1)",  
            "1x2": "Dreiachser (1+2) - Tandem",
            "2x2": "Vierachser (2+2)",
            "1x3": "Vierachser (1+3) - Tridem",
            "3x1": "Vierachser (3+1)",
        }
        
        if konfiguration in typ_mapping:
            return typ_mapping[konfiguration]
        else:
            # Fallback basierend auf Achsenanzahl
            if anzahl_achsen == 2:
                return f"Zweiachser ({konfiguration})"
            elif anzahl_achsen == 3:
                return f"Dreiachser ({konfiguration})"
            elif anzahl_achsen == 4:
                return f"Vierachser ({konfiguration})"
            elif anzahl_achsen >= 5:
                return f"Mehrachser ({konfiguration})"
            else:
                return f"Unbekannt ({konfiguration})"
            
    def klassifiziere_bus_typ(self, konfiguration, anzahl_achsen):
        """
        Klassifiziert Bus-Typ basierend auf Achskonfiguration.
        
        Args:
            konfiguration: String wie "1x1", "1x2", "2x2" etc.
            anzahl_achsen: Gesamtanzahl der Achsen
            
        Returns:
            String mit Bus-Typ
        """
        # Mapping von Konfiguration zu Typ
        typ_mapping = {
            "1x1": "Bus (E, E) 120",
            "1x2": "Bus (E, Dp) 121",  
            "1x1x1": "Gelenkbus (E, E + E) 280",
            "1x1x2": "Gelenkbus (E,E + Dp) 281",
        }
        
        if konfiguration in typ_mapping:
            return typ_mapping[konfiguration]
        else:
            # Fallback basierend auf Achsenanzahl
            if anzahl_achsen == 2:
                return f"Standardbus ({konfiguration})"
            elif anzahl_achsen == 3:
                return f"Gelenkbus ({konfiguration})"
            elif anzahl_achsen >= 4:
                return f"Doppelgelenkbus ({konfiguration})"
            else:
                return f"Unbekannt ({konfiguration})"
            
    def klassifiziere_anhaenger_typ(self, konfiguration, anzahl_achsen):
        """
        Klassifiziert Anhänger-Typ basierend auf Achskonfiguration.
        
        Args:
            konfiguration: String wie "1, 2, 3, 1x1, 1x2, 2x2" etc.
            anzahl_achsen: Gesamtanzahl der Achsen
            
        Returns:
            String mit Anhänger-Typ
        """

        typ_mapping = {
            "1": "E",           # 1 Achse
            "2": "Dp",          # 2 Achsen (Tandem)
            "3": "Dr",          # 3 Achsen (Tridem)
            "1x1": "E, E",        # 2 Achsen mit Abstand (1x1)
            "1x2": "E, Dp",       # 3 Achsen mit Abstand (1x2)
            "1x3": "E, Dr",       # 4 Achsen mit Abstand (1x3)
            "2x2": "Dp, Dp",      # 4 Achsen (2x2)
        }

        if konfiguration in typ_mapping:
            return typ_mapping[konfiguration]
        else:
            # Fallback basierend auf Achsenanzahl
            if anzahl_achsen == 1:
                return f"Einachs-Anhänger ({konfiguration})"
            elif anzahl_achsen == 2:
                return f"Zweiachs-Anhänger ({konfiguration})"
            elif anzahl_achsen == 3:
                return f"Dreiachs-Anhänger ({konfiguration})"
            elif anzahl_achsen == 4:
                return f"Vierachs-Anhänger ({konfiguration})"
            elif anzahl_achsen >= 5:
                return f"Mehrachs-Anhänger ({konfiguration})"
            else:
                return f"Unbekannter Anhänger ({konfiguration})"
    
    def classify_objects(self, distanz_schwellwert=0.07):
        """
        Klassifiziert Fahrzeuge basierend auf Achskonfiguration. 
        
        Args:
            distanz_schwellwert:  Schwellwert für Achsgruppenerkennung
            
        Returns: 
            Dictionary mit vehicle_id als Schlüssel und Klassifikations-Info als Wert
        """
        classification = {}
        
        for v_id, wheels in self.associations.items():
            if not wheels:
                continue
                
            vehicle = next((v for v in self.Pkw + self.Lkw + self.Anhaenger + self.Bus + self.Transporter + self.Traktor if v[2] == v_id), None)
            if vehicle is not None:
                label = vehicle[3]
            else:
                label = None
            
            # Extrahiere Rad-IDs
            wheel_ids = [w["track_id"] for w in wheels]
            num_wheels = len(wheel_ids)
            
            # Analysiere Achskonfiguration
            achsen_analyse = self.analysiere_achskonfiguration(wheel_ids, distanz_schwellwert)
            
            if label == "car":
                classification[v_id] = {
                    "typ": "Auto",
                    "anzahl_raeder": num_wheels,
                    "anzahl_achsen": achsen_analyse["anzahl_achsen"],
                    "konfiguration": achsen_analyse["konfiguration"],
                    "rad_ids": wheel_ids,
                    "achsgruppen": achsen_analyse["achsgruppen"],
                    "distanzen": achsen_analyse["distanzen"]
                }
            elif label == "truck":
                # Detaillierte Klassifikation basierend auf Achskonfiguration
                lkw_typ = self.klassifiziere_lkw_typ(
                    achsen_analyse["konfiguration"],
                    achsen_analyse["anzahl_achsen"]
                )
                
                classification[v_id] = {
                    "typ": lkw_typ,
                    "anzahl_raeder": num_wheels,
                    "anzahl_achsen": achsen_analyse["anzahl_achsen"],
                    "konfiguration": achsen_analyse["konfiguration"],
                    "rad_ids": wheel_ids,
                    "achsgruppen": achsen_analyse["achsgruppen"],
                    "distanzen": achsen_analyse["distanzen"]
                }
            elif label == "van":
                classification[v_id] = {
                    "typ": "Transporter",
                    "anzahl_raeder": num_wheels,
                    "anzahl_achsen": achsen_analyse["anzahl_achsen"],
                    "konfiguration": achsen_analyse["konfiguration"],
                    "rad_ids": wheel_ids,
                    "achsgruppen": achsen_analyse["achsgruppen"],
                    "distanzen": achsen_analyse["distanzen"]
                }
            elif label == "bus":
                bus_typ = self.klassifiziere_bus_typ(
                    achsen_analyse["konfiguration"],
                    achsen_analyse["anzahl_achsen"]
                )
                classification[v_id] = {
                    "typ": bus_typ,
                    "anzahl_raeder": num_wheels,
                    "anzahl_achsen": achsen_analyse["anzahl_achsen"],
                    "konfiguration": achsen_analyse["konfiguration"],
                    "rad_ids": wheel_ids,
                    "achsgruppen": achsen_analyse["achsgruppen"],
                    "distanzen": achsen_analyse["distanzen"]
                }
            elif label == "tractor":
                classification[v_id] = {
                    "typ": "Traktor",
                    "anzahl_raeder": num_wheels,
                    "anzahl_achsen": achsen_analyse["anzahl_achsen"],
                    "konfiguration": achsen_analyse["konfiguration"],
                    "rad_ids": wheel_ids,
                    "achsgruppen": achsen_analyse["achsgruppen"],
                    "distanzen": achsen_analyse["distanzen"]
                }
            elif label == "trailer":
                # Detaillierte Klassifikation basierend auf Achskonfiguration
                anhaenger_typ = self.klassifiziere_anhaenger(
                    achsen_analyse["konfiguration"],
                    achsen_analyse["anzahl_achsen"]
                )
                
                classification[v_id] = {
                    "typ": anhaenger_typ,
                    "anzahl_raeder": num_wheels,
                    "anzahl_achsen": achsen_analyse["anzahl_achsen"],
                    "konfiguration": achsen_analyse["konfiguration"],
                    "rad_ids": wheel_ids,
                    "achsgruppen": achsen_analyse["achsgruppen"],
                    "distanzen": achsen_analyse["distanzen"]
                }
            else:
                classification[v_id] = {
                    "typ": "Unbekannt",
                    "anzahl_raeder": num_wheels,
                    "anzahl_achsen": achsen_analyse["anzahl_achsen"],
                    "konfiguration": achsen_analyse["konfiguration"],
                    "rad_ids": wheel_ids,
                    "achsgruppen": achsen_analyse["achsgruppen"],
                    "distanzen": achsen_analyse["distanzen"]
                }
        
        return classification