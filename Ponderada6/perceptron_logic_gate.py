import numpy as np
import re

class Perceptron:
    def __init__(self, learning_rate=0.1, n_iterations=100, threshold=0.5):
        self.learning_rate = learning_rate
        self.n_iterations = n_iterations
        self.threshold = threshold
        self.weights = np.zeros(2)
        self.bias = 0

    def activation_function(self, x):
        return 1 if x >= self.threshold else 0

    def predict(self, inputs):
        # Calcula a soma ponderada das entradas
        linear_output = np.dot(inputs, self.weights) + self.bias
        # Aplica a função degrau para determinar a saída
        y_predicted = self.activation_function(linear_output)
        return y_predicted

    def train(self, X, y):
        for _ in range(self.n_iterations):
            for x, y_true in zip(X, y):
                y_pred = self.predict(x)
                error = y_true - y_pred
                self.weights += error * self.learning_rate * x
                self.bias += error * self.learning_rate


X = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])

logic_gate_dict = {
    r"\b([Aa][Nn][Dd])":[0, 0, 0, 1],
    r"\b([Oo][Rr])":[0, 1, 1, 1],
    r"\b([Nn][Aa][Nn][Dd])":[1, 1, 1, 0],
    r"\b([Xx][Oo][Rr])":[0, 1, 1, 0]
}

def main():
    logic_gate = input("Para qual porta lógica você gostaria de treinar o perceptron? ")
    for key, actions in logic_gate_dict.items():
        key_re = re.compile(key)
        if key_re.findall(logic_gate) == ["xor"]:
            print("Xor é uma execeção, leia a documentação para entender melhor sobre este caso, percéptrons e o inverno da IA.")
        elif key_re.findall(logic_gate):
            y = actions

            # Treinando o Perceptron
            perceptron = Perceptron()
            perceptron.train(X, y)

            # Testando o Perceptron
            for inputs in X:
                print(f"in {inputs}, out: {perceptron.predict(inputs)}")



# Exemplo de uso
if __name__ == "__main__":
    main()


