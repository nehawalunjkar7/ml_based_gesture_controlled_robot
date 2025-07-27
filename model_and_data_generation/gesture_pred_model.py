import pandas as pd
import glob
import numpy as np
import joblib
import os
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder, StandardScaler
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report
from sklearn.metrics import accuracy_score
from sklearn.model_selection import GridSearchCV

def train_model():
    # Load and combine CSVs
    csv_files = glob.glob('data/*.csv')
    df_list = [pd.read_csv(file, header=None) for file in csv_files]  
    data = pd.concat(df_list, ignore_index=True) 

    # Separate features and labels
    X = data.iloc[:, :-1].values  # 63 features
    y = data.iloc[:, -1].values   # labels

    # Encode gesture labels (e.g. 'Index and middle finger up' -> Increase speed)
    le = LabelEncoder()
    y_encoded = le.fit_transform(y)

    # Normalize features
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)

    # Split data
    X_train, X_test, y_train, y_test = train_test_split(X_scaled, y_encoded, test_size=0.2, random_state=51, shuffle=True)

    param_grid = {
        'n_estimators': [100, 200],
        'max_depth': [10, 20, None],
        'min_samples_split': [2, 5],
    }

    grid = GridSearchCV(RandomForestClassifier(), param_grid, cv=5, verbose=3)
    grid.fit(X_train, y_train)

    best_params = grid.best_params_

    rf = RandomForestClassifier(
        **best_params,
        random_state=42,
        oob_score=True,
        class_weight='balanced'      
    )

    rf = grid.best_estimator_
    rf.oob_score = True  # Add this manually if required
    rf.class_weight = 'balanced'
    rf.fit(X_train, y_train)

    y_pred = rf.predict(X_test)
    accuracy = accuracy_score(y_test, y_pred) #precision, recall, F1-score on the predictions.
    print(f"Test accuracy: {accuracy}") 
    
    print(f"OOB score: {rf.oob_score_}")
    print(classification_report(y_test, y_pred))

    # Save models
    os.makedirs("models", exist_ok=True)
    joblib.dump(rf, 'models/gesture_classifier.joblib')
    joblib.dump(le, 'models/label_encoder.joblib')
    joblib.dump(scaler, 'models/scaler.joblib')
    print("[INFO] Models saved to 'models/' directory.")

if __name__ == "__main__":
    train_model()