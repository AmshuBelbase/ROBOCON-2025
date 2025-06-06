import torch

# Check if MPS backend is available
if torch.backends.mps.is_available():
    print("MPS is available!")
    device = torch.device("mps")
else:
    print("MPS is not available. Using CPU.")
    device = torch.device("cpu")