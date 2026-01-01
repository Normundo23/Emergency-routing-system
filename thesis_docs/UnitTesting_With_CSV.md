# Unit Testing & Validation: Incident Prediction Model
**Component:** Decision Tree Classifier for Risk Assessment
**Date:** January 1, 2026

---

## 1. Overview
This document details the validation of the Machine Learning component using a CSV-based testing approach. We utilize **PyTest** to load historical traffic data, train a Decision Tree model on-the-fly, and verify its logical consistency against known traffic scenarios.

## 2. Dataset Structure
**File:** `emergency_incidents.csv`
This dataset represents historical traffic conditions. We use this to train the model before testing it.

| Feature | Description | Example Values |
| :--- | :--- | :--- |
| **TimeOfDay** | Hour in 24h format | 8 (8 AM), 17 (5 PM) |
| **WeatherCondition** | Binary Flag | 1 = Rain, 0 = Clear |
| **RoadType** | Categorical ID | 0 = Highway, 1 = City, 2 = Rural |
| **TrafficVolume** | Vehicles per Hour | 1200, 50 |
| **RiskLevel (Target)** | Classification | **1 = High Risk**, 0 = Low Risk |

### Sample Data Content
```csv
TimeOfDay,WeatherCondition,RoadType,TrafficVolume,RiskLevel
8,1,0,1200,1
14,0,1,300,0
17,1,0,1500,1
3,0,1,50,0
... (27 records total)
```

---

## 3. Test Methodology (PyTest)
We developed a test script `test_incident_prediction.py` that follows a strict **"Fixture -> Train -> Verify"** workflow.

### 3.1 The Workflow
1.  **Fixture Setup:** PyTest loads `emergency_incidents.csv`.
2.  **Training:** It splits the data (70% Train, 30% Test) and fits a `DecisionTreeClassifier`.
3.  **Unit Tests:** It feeds specific "mock" scenarios (e.g., "Rainy Rush Hour") to the trained model and asserts the output matches the expected Risk Level.
4.  **Eval Tests:** It calculates Accuracy, Precision, and Recall on the remaining 30% test data.

### 3.2 Python Test Script Code
```python
@pytest.fixture(scope="module")
def model_and_data():
    df = pd.read_csv("emergency_incidents.csv")
    # ... Training Logic ...
    return clf, X_test, y_test

def test_TC01_high_risk_scenario(model_and_data):
    """
    Scenario: Rush Hour (8am) + Rain (1) + Highway (0) + High Traffic (1200)
    Expected: High Risk (1)
    """
    clf, _, _ = model_and_data
    incident = np.array([[8, 1, 0, 1200]]) 
    prediction = clf.predict(incident)[0]
    assert prediction == 1  # PASS
```

---

## 4. Test Results (Evidence)
The following output confirms that the Decision Tree correctly learned the traffic rules from the CSV file and passed all accuracy benchmarks (>75%).

```text
============================= test session starts =============================
platform win32 -- Python 3.12.10, pytest-9.0.2, pluggy-1.6.0
rootdir: C:\Users\denni\OneDrive\Desktop\Emergency routing system
collected 6 items

test_incident_prediction.py::test_TC01_high_risk_scenario PASSED         [ 16%]
test_incident_prediction.py::test_TC02_low_risk_scenario PASSED          [ 33%]
test_incident_prediction.py::test_TC03_edge_case_rainy_night PASSED      [ 50%]
test_incident_prediction.py::test_TC09_accuracy PASSED                   [ 66%]
test_incident_prediction.py::test_TC10_precision_recall PASSED           [ 83%]
test_incident_prediction.py::test_TC11_f1_score PASSED                   [100%]

============================== 6 passed in 2.21s ==============================
```

## 5. Conclusion
The unit tests verify that the Decision Tree algorithm correctly classifies dynamic traffic situations.
*   **Logical Correctness:** The model correctly identified High Risk in TC01 and Low Risk in TC02.
*   **Performance:** The model achieved high accuracy on the hold-out test set, validating its use in the live routing engine.
