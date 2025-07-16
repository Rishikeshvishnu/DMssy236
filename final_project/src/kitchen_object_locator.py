import numpy as np
from sklearn.tree import DecisionTreeClassifier, export_text
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import confusion_matrix, classification_report
import json
import os

# Trying with a different classifier
from sklearn.ensemble import RandomForestClassifier
from collections import Counter

class KitchenObjectLocator:
    def __init__(self):
        # Hmm, keeping the original categories since they work well...
        self.weight_categories = {
            'light': 0,    # < 0.5 kg
            'medium': 1,   # 0.5 - 2 kg
            'heavy': 2     # > 2 kg
        }
        
        self.size_categories = {
            'small': 0,    # < 20cm in any dimension
            'medium': 1,   # 20-50cm in any dimension
            'large': 2     # > 50cm in any dimension
        }

        self.temp_sensitivity = {
            'room_temp': 0,    # Stable at room temperature
            'perishable': 1,   # Needs refrigeration
            'frozen': 2        # Needs freezing
        }

        self.container_types = {
            'none': 0,         # Not a container
            'open': 1,         # Open container (plate, bowl)
            'closed': 2        # Closed container (bottle, jar)
        }

        self.usage_frequency = {
            'rare': 0,         # Used rarely
            'occasional': 1,   # Used occasionally
            'frequent': 2      # Used frequently
        }
        
        self.object_encoder = LabelEncoder()
        self.location_encoder = LabelEncoder()

        # DecisionTreeClassifier
        # self.clf = DecisionTreeClassifier(max_depth=5, min_samples_split=2, random_state=42)

        # RandomForestClassifier
        self.clf = RandomForestClassifier(n_estimators=100, max_depth=4, random_state=42)
        
        self._initialize_training_data()
    
    def _initialize_training_data(self):
        """
        Initialize training data with aligned locations and consistent object characteristics
        [object, weight, size, is_edible, temp_sensitive, container_type, usage_freq, location]
        """
        # self.training_data = [
        #     Table items - keeping these mostly the same
        #     ['beer', 1, 1, 1, 0, 2, 1, 'table'],          # Medium, edible, room temp, closed container, occasional
        #     ['bottle', 1, 1, 1, 0, 2, 2, 'table'],         # Medium, edible, room temp, closed container, frequent
        #     ['water', 1, 1, 1, 0, 2, 2, 'table'],         # Medium, edible, room temp, closed container, frequent
        #     ['glass', 0, 0, 0, 0, 1, 2, 'table'],         # Light, not edible, room temp, open container, frequent
        #     ['cup', 0, 0, 0, 0, 1, 2, 'table'],           # Light, not edible, room temp, open container, frequent
            
        #     Cupboard items - unchanged as they're correct
        #     ['plate', 0, 1, 0, 0, 1, 2, 'cupboard'],      # Light, not edible, room temp, open container, frequent
        #     ['bowl', 1, 0, 0, 0, 1, 2, 'cupboard'],       # Medium, not edible, room temp, open container, frequent
        #     ['mug', 0, 0, 0, 0, 1, 2, 'cupboard'],        # Light, not edible, room temp, open container, frequent
        #     ['pot', 2, 1, 0, 0, 1, 1, 'cupboard'],        # Heavy, not edible, room temp, open container, occasional
        #     ['pan', 2, 1, 0, 0, 1, 1, 'cupboard'],        # Heavy, not edible, room temp, open container, occasional
            
        #     Counter items - updated location from cafe_table
        #     ['bread', 0, 1, 1, 0, 0, 2, 'counter'],       # Light, edible, room temp, no container, frequent
        #     ['fruit', 0, 0, 1, 0, 0, 2, 'counter'],       # Light, edible, room temp, no container, frequent
        #     ['vegetables', 1, 1, 1, 0, 0, 2, 'counter'],  # Medium, edible, room temp, no container, frequent
            
        #     Trash items - unchanged as they're correct
        #     ['trash', 1, 1, 0, 0, 0, 2, 'trash_can'],     # Medium, not edible, room temp, no container, frequent
        #     ['waste', 1, 1, 0, 0, 0, 2, 'trash_can']      # Medium, not edible, room temp, no container, frequent
        # ]
        
        self.training_data = [
        # Balanced training set with new object_type feature
        ['beer', 1, 1, 1, 0, 2, 1, self.get_object_type('beer'), 'table'],
        ['bottle', 1, 1, 1, 0, 2, 2, self.get_object_type('bottle'), 'table'],
        ['water', 1, 1, 1, 0, 2, 2, self.get_object_type('water'), 'table'],
        ['glass', 0, 0, 0, 0, 1, 2, self.get_object_type('glass'), 'table'],
        ['plate', 0, 1, 0, 0, 1, 2, self.get_object_type('plate'), 'cupboard'],
        ['bowl', 1, 0, 0, 0, 1, 2, self.get_object_type('bowl'), 'cupboard'],
        ['mug', 0, 0, 0, 0, 1, 2, self.get_object_type('mug'), 'cupboard'],
        ['pot', 2, 1, 0, 0, 1, 1, self.get_object_type('pot'), 'cupboard'],
        ['pan', 2, 1, 0, 0, 1, 1, self.get_object_type('pan'), 'cupboard'],
        ['bread', 0, 1, 1, 0, 0, 2, self.get_object_type('bread'), 'counter'],
        ['fruit', 0, 0, 1, 0, 0, 2, self.get_object_type('fruit'), 'counter'],
        ['vegetables', 1, 1, 1, 0, 0, 2, self.get_object_type('vegetables'), 'counter'],
        ['trash', 1, 1, 0, 0, 0, 2, self.get_object_type('trash'), 'trash_can'],
        ['waste', 1, 1, 0, 0, 0, 2, self.get_object_type('waste'), 'trash_can']
    ]

        # Prepare training data - keeping the same structure
        X = []
        y = []
        objects = []
        locations = []
        
        for item in self.training_data:
            objects.append(item[0])
            # Features: weight, size, is_edible, temp_sensitive, container_type, usage_freq,  object_type
            X.append(item[1:8])
            locations.append(item[8])
        
        self.object_encoder.fit(objects)
        self.location_encoder.fit(locations)
        
        self.X_train = np.array(X)
        self.y_train = self.location_encoder.transform(locations)

        # For Debugging
        print("Training set class distribution:", Counter(self.y_train))
        print("Label mapping:", dict(enumerate(self.location_encoder.classes_)))

        # Using RandomForest
        self.clf = RandomForestClassifier(n_estimators=30, max_depth=3, random_state=42)
        self.clf.fit(self.X_train, self.y_train)
        
        # # Print decision tree structure for debugging
        # tree_rules = export_text(self.clf, feature_names=[
        #     'weight', 'size', 'edible', 'temp_sens', 'container', 'usage_freq'
        # ])
        # print("\nDecision Tree Rules:")
        # print(tree_rules)
    
    # New Function to Get Object Type (prevents similar objects (pan, pot, trash) from confusing the model)
    def get_object_type(self, object_name):

        """Assigns object type based on name"""

        cooking_utensils = {"pot", "pan"}
        storage_containers = {"bottle", "jar"}
        waste_items = {"trash", "waste"}
        dishware = {"plate", "bowl", "glass", "mug"}

        if object_name in cooking_utensils:
            return 0
        elif object_name in storage_containers:
            return 1
        elif object_name in waste_items:
            return 2
        elif object_name in dishware:
            return 3
        else:
            return 1  # Default to a general category


    def predict_location(self, object_name, weight_category=1, size_category=1, 
                        is_edible=0, temp_sensitive=0, container_type=0, usage_freq=1):
        """
        Predict location with validation checks maintained
        """
        # Input validation - keeping all checks
        for val, name, valid_range in [
            (weight_category, "Weight", [0,1,2]),
            (size_category, "Size", [0,1,2]),
            (is_edible, "Edible", [0,1]),
            (temp_sensitive, "Temperature sensitivity", [0,1,2]),
            (container_type, "Container type", [0,1,2]),
            (usage_freq, "Usage frequency", [0,1,2])
        ]:
            if val not in valid_range:
                raise ValueError(f"{name} must be in range {valid_range}")
        
        # For Debugging
        print(f"Predicting for object: {object_name}")
        print(f"Features: weight={weight_category}, size={size_category}, edible={is_edible}, "
          f"temp_sensitivity={temp_sensitive}, container={container_type}, freq={usage_freq}")

    
        features = np.array([[
            weight_category, size_category, is_edible,
            temp_sensitive, container_type, usage_freq,
            self.get_object_type(object_name)  # Added object type to the feature set
        ]])
        
        #location_encoded = self.clf.predict(features)[0]
        #return self.location_encoder.inverse_transform([location_encoded])[0]

        # For Debugging
        print("Encoded feature array:", features)

        location_encoded = self.clf.predict(features)[0]
        print("Raw model output (encoded location):", location_encoded)

        predicted_location = self.location_encoder.inverse_transform([location_encoded])[0]
        print("Decoded predicted location:", predicted_location)
        
        return predicted_location

    
    def evaluate_model(self):
        """Evaluate model performance - keeping evaluation logic intact"""
        y_pred = self.clf.predict(self.X_train)
        
        y_true_names = self.location_encoder.inverse_transform(self.y_train)
        y_pred_names = self.location_encoder.inverse_transform(y_pred)
        
        print("\nClassification Report:")
        print(classification_report(y_true_names, y_pred_names))
        
        print("\nConfusion Matrix:")
        cm = confusion_matrix(y_true_names, y_pred_names)
        locations = self.location_encoder.classes_
        print("\t" + "\t".join(locations))
        for i, row in enumerate(cm):
            print(f"{locations[i]}\t" + "\t".join(str(x) for x in row))

def main():
    locator = KitchenObjectLocator()
    
    # Updated test cases to cover all locations
    test_objects = [
        # name, weight, size, edible, temp_sens, container, usage
        ('water', 1, 1, 1, 0, 2, 2),     # Should predict: table
        ('plate', 0, 1, 0, 0, 1, 2),     # Should predict: cupboard
        ('bread', 0, 1, 1, 0, 0, 2),     # Should predict: counter
        ('trash', 1, 1, 0, 0, 0, 2)      # Should predict: trash_can
    ]
    
    print("\nTesting predictions:")
    for obj, *features in test_objects:
        location = locator.predict_location(obj, *features)
        print(f"{obj.capitalize()} is likely to be found in/on the {location}")
    
    locator.evaluate_model()

if __name__ == "__main__":
    main()