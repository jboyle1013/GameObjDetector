import json
import os
from PIL import Image
from ultralytics import YOLO

# Load a pretrained YOLOv8n model
model = YOLO("train13/weights/best.pt")

# Load ground truth labels from JSON
with open('BenchMarkImages/ObjsInImage.json', 'r') as file:
    ground_truths = json.load(file)

# Initialize variables for accuracy calculation
total_predictions = 0
correct_predictions = 0
predictions = {}


# object classes
classNames = ['BigBox', 'BlueZone', 'Button', 'GreenZone', 'Nozzle', 'RedZone',
              'Rocket', 'SmallBox', 'StartZone', 'WhiteLine', 'YellowLine']
for _, image in enumerate(os.listdir("BenchMarkImages/Images")):
    predicted_labels = []
    # Run inference on 'bus.jpg'
    results = model(f"BenchMarkImages/Images/{image}")  # results list

    # Show the results
    for r in results:
        im_array = r.plot()  # plot a BGR numpy array of predictions
        im = Image.fromarray(im_array[..., ::-1])  # RGB PIL image
        # im.show()  # show image
        im.save(f'BenchMarkImages/results/results{_+1}.jpeg')  # save image
        predictions[image.split('.')[0]] = []
        for b in r.boxes:
            class_name = classNames[int(b.cls[0])]
            predictions[image.split('.')[0]].append(class_name)

for _, image in enumerate(os.listdir("BenchMarkImages/Images")):
    # Compare with ground truth and count correct predictions
    for label in predictions[image.split('.')[0]]:
        if label in ground_truths[image.split('.')[0]]:
            correct_predictions += 1
            ground_truths[image.split('.')[0]].remove(label)  # Remove to avoid double counting

    total_predictions += len(predictions[image.split('.')[0]])

print(correct_predictions)
# Calculate overall accuracy
accuracy = (correct_predictions / total_predictions) * 100 if total_predictions > 0 else 0
print(f"Model Accuracy: {accuracy}%")







