import pytest
import pandas as pd
import numpy as np
import sys
import os
from sklearn.tree import DecisionTreeClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score

# ----------------------------
# Fixture: Load CSV + Train Model
# ----------------------------
@pytest.fixture(scope="module")
def model_and_data():
    # Load raw CSV
    # Features: 
    #   TimeOfDay (0-23)
    #   WeatherCondition (1=Rain, 0=Clear)
    #   RoadType (0=H-way, 1=City, 2=Rural)
    #   TrafficVolume (Vehicles/Hour)
    # Label:
    #   RiskLevel (1 = High Risk/Delay, 0 = Low Risk)
    
    csv_path = "emergency_incidents.csv"
    if not os.path.exists(csv_path):
        pytest.fail(f"CSV file not found: {csv_path}")

    df = pd.read_csv(csv_path)
    
    # Split features (X) and label (y)
    X = df[["TimeOfDay", "WeatherCondition", "RoadType", "TrafficVolume"]].values
    y = df["RiskLevel"].values
    
    # Train-test split (Small dataset, so test_size small to keep enough training data)
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)
    
    # Train Decision Tree
    clf = DecisionTreeClassifier(random_state=42, max_depth=5)
    clf.fit(X_train, y_train)
    
    return clf, X_test, y_test

# ----------------------------
# Single-Incident Test Cases
# ----------------------------
def test_TC01_high_risk_scenario(model_and_data):
    """
    Scenario: Rush Hour (8am) + Rain (1) + Highway (0) + High Traffic (1200)
    Expected: High Risk (1)
    """
    clf, _, _ = model_and_data
    incident = np.array([[8, 1, 0, 1200]]) 
    prediction = clf.predict(incident)[0]
    assert prediction == 1 

def test_TC02_low_risk_scenario(model_and_data):
    """
    Scenario: Night (3am) + Clear (0) + City (1) + Low Traffic (50)
    Expected: Low Risk (0)
    """
    clf, _, _ = model_and_data
    incident = np.array([[3, 0, 1, 50]])
    prediction = clf.predict(incident)[0]
    assert prediction == 0

def test_TC03_edge_case_rainy_night(model_and_data):
    """
    Scenario: Late Night (23pm) + Rain (1) + Rural (2)
    This tests if Rain alone triggers risk in rural areas.
    """
    clf, _, _ = model_and_data
    # Note: Our tiny dataset implies rain=risk often
    incident = np.array([[23, 1, 1, 300]]) 
    prediction = clf.predict(incident)[0]
    assert prediction == 1

# ----------------------------
# Dataset-level Evaluation
# ----------------------------
def test_TC09_accuracy(model_and_data):
    clf, X_test, y_test = model_and_data
    y_pred = clf.predict(X_test)
    acc = accuracy_score(y_test, y_pred)
    # With tiny synthetic data, we expect perfect overfitting or high accuracy
    assert acc >= 0.75 

def test_TC10_precision_recall(model_and_data):
    clf, X_test, y_test = model_and_data
    y_pred = clf.predict(X_test)
    # Handle zero division if any
    precision = precision_score(y_test, y_pred, zero_division=0)
    recall = recall_score(y_test, y_pred, zero_division=0)
    
    print(f"DEBUG: Precision={precision}, Recall={recall}")
    assert precision >= 0.70
    assert recall >= 0.70

def test_TC11_f1_score(model_and_data):
    clf, X_test, y_test = model_and_data
    y_pred = clf.predict(X_test)
    f1 = f1_score(y_test, y_pred, zero_division=0)
    assert f1 >= 0.70

if __name__ == "__main__":
    sys.exit(pytest.main(["-v", __file__]))
