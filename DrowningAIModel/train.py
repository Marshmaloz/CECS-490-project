import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
import albumentations as A
from albumentations.pytorch import ToTensorV2
import cv2
import numpy as np
import os
import joblib
from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
from tqdm import tqdm
from PIL import Image

# 1. Define the CustomCNN Architecture (must match DrownDetect.py)
class CustomCNN(nn.Module):
    def __init__(self, num_classes):
        super(CustomCNN, self).__init__()
        self.conv1 = nn.Conv2d(3, 16, 5)
        self.conv2 = nn.Conv2d(16, 32, 5)
        self.conv3 = nn.Conv2d(32, 64, 3)
        self.conv4 = nn.Conv2d(64, 128, 5)
        self.fc1 = nn.Linear(128, 256)
        self.fc2 = nn.Linear(256, num_classes)
        self.pool = nn.MaxPool2d(2, 2)

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = self.pool(F.relu(self.conv3(x)))
        x = self.pool(F.relu(self.conv4(x)))
        bs, _, _, _ = x.shape
        x = F.adaptive_avg_pool2d(x, 1).reshape(bs, -1)
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

# 2. Custom Dataset Class
class DrowningDataset(Dataset):
    def __init__(self, image_paths, labels, transform=None):
        self.image_paths = image_paths
        self.labels = labels
        self.transform = transform

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        image = cv2.imread(self.image_paths[idx])
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        label = self.labels[idx]

        if self.transform:
            image = self.transform(image=image)["image"]

        return image, torch.tensor(label, dtype=torch.long)

# 3. Training Script Parameters
DATA_DIR = 'dataset' # Update this to your folder path
BATCH_SIZE = 32
LR = 0.001
EPOCHS = 100
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

def train():
    # Load and organize image paths and labels
    image_paths = []
    labels = []
    print(f"Scanning {DATA_DIR} for training data...")
    
    for class_name in os.listdir(DATA_DIR):
        class_dir = os.path.join(DATA_DIR, class_name)
        if os.path.isdir(class_dir):
            for img_name in os.listdir(class_dir):
                if img_name.lower().endswith(('.png', '.jpg', '.jpeg')):
                    image_paths.append(os.path.join(class_dir, img_name))
                    labels.append(class_name)

    if not image_paths:
        print(f"Error: No images found in {DATA_DIR}. Please check your folder structure.")
        return

    # Binarize labels
    lb = LabelBinarizer()
    labels = lb.fit_transform(labels)
    if len(lb.classes_) == 2:
        # For 2 classes, LabelBinarizer returns a 1D array of 0/1, we need to convert for multi-class cross entropy
        labels = np.column_stack((1 - labels, labels))
    labels = np.argmax(labels, axis=1)

    # Save LabelBinarizer
    joblib.dump(lb, 'lb.pkl')
    print(f"Saved label binarizer to lb.pkl for classes: {lb.classes_}")

    # Split data
    train_paths, val_paths, train_labels, val_labels = train_test_split(
        image_paths, labels, test_size=0.2, random_state=42
    )

    # Augmentations (matching DrownDetect.py resize)
    train_transform = A.Compose([
        A.Resize(224, 224),
        A.HorizontalFlip(p=0.5),
        A.RandomBrightnessContrast(p=0.2),
        A.Rotate(limit=15),
        A.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
        ToTensorV2()
    ])

    val_transform = A.Compose([
        A.Resize(224, 224),
        A.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
        ToTensorV2()
    ])

    # DataLoaders
    train_dataset = DrowningDataset(train_paths, train_labels, transform=train_transform)
    val_dataset = DrowningDataset(val_paths, val_labels, transform=val_transform)

    train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=False)

    # Model Setup
    model = CustomCNN(len(lb.classes_)).to(DEVICE)
    optimizer = optim.Adam(model.parameters(), lr=LR)
    criterion = nn.CrossEntropyLoss()

    # Training Loop
    for epoch in range(EPOCHS):
        model.train()
        train_loss = 0
        for images, targets in tqdm(train_loader, desc=f"Epoch {epoch+1}/{EPOCHS}"):
            images, targets = images.to(DEVICE), targets.to(DEVICE)
            
            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, targets)
            loss.backward()
            optimizer.step()
            train_loss += loss.item()

        # Validation Loop
        model.eval()
        val_loss = 0
        correct = 0
        with torch.no_grad():
            for images, targets in val_loader:
                images, targets = images.to(DEVICE), targets.to(DEVICE)
                outputs = model(images)
                loss = criterion(outputs, targets)
                val_loss += loss.item()
                _, predicted = torch.max(outputs, 1)
                correct += (predicted == targets).sum().item()

        print(f"Train Loss: {train_loss/len(train_loader):.4f} | Val Loss: {val_loss/len(val_loader):.4f} | Val Acc: {100 * correct / len(val_dataset):.2f}%")

    # Save final model
    torch.save(model.state_dict(), 'model.pth')
    print("Training complete. Model saved as model.pth")

if __name__ == '__main__':
    train()
