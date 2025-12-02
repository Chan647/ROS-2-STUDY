import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LogisticRegression
from sklearn.tree import DecisionTreeClassifier
from sklearn.tree import plot_tree


def main():
	df = pd.read_csv('/home/chan/december_ws/lidar_data.csv')
	feature_cols = [f"range_{i}" for i in range(360)]
	df.columns = feature_cols + ["situation"]

	data = df[feature_cols]	
	target = df['situation']

	train_input, test_input, train_target, test_target = train_test_split(data, target, test_size=0.2, random_state=100)
	ss = StandardScaler()
	ss.fit(train_input)
	train_scaled = ss.transform(train_input)
	test_scaled = ss.transform(test_input)

	lr = LogisticRegression()
	lr.fit(train_scaled, train_target)
	lr.fit(train_scaled, train_target)
	print(lr.score(train_scaled, train_target))
	print(lr.score(test_scaled, test_target))

	dt = DecisionTreeClassifier(max_depth=3, random_state=100)
	dt.fit(train_scaled, train_target)
	print(dt.score(train_scaled, train_target))
	print(dt.score(test_scaled, test_target))

	plt.figure(figsize=(20,15))
	plot_tree(dt, filled=True, feature_names=feature_cols)
	plt.show()

if __name__ == '__main__':
    main()