import os
import torch
import pickle

def to_cpu(obj):
    if isinstance(obj, torch.Tensor):  # Only for torch.Tensor execute .to("cpu") operation
        return obj.to("cpu")
    elif isinstance(obj, dict):  # Recursively process dictionary types
        return {k: to_cpu(v) for k, v in obj.items()}
    elif isinstance(obj, list):  # Recursively process list types
        return [to_cpu(v) for v in obj]
    else:  # Other types, no need to process, return directly
        return obj

def convert_model_for_cpu(model_path):
    print(f"Model being converted: {model_path}")
    
    try:
        # Determine the file type
        # if model_path.endswith(".pt") or model_path.endswith(".pth"):
        #     model = torch.load(model_path)
        #     model = to_cpu(model)
        if model_path.endswith(".jit"):
            model = torch.jit.load(model_path)
            model.to("cpu")
        elif model_path.endswith(".pkl"):
            with open(model_path, "rb") as f:
                model = pickle.load(f)
            model = to_cpu(model)
    except Exception as e:
        print(f"Skip if you encounter a problem {model_path}，reason: {e}")
        return

    # Save the converted model and add the file name'_cpu' as a distinction
    base_path, ext = os.path.splitext(model_path)
    cpu_model_path = base_path + '_cpu' + ext

    if model_path.endswith(".jit"):
        torch.jit.save(model, cpu_model_path)
    elif model_path.endswith(".pt") or model_path.endswith(".pth"):
        torch.save(model, cpu_model_path)
    elif model_path.endswith(".pkl"):
        with open(cpu_model_path, 'wb') as f:
            pickle.dump(model, f)

    print(f"Complete the conversion and save to: {cpu_model_path}")
    
# Search for all files in the directory and convert them
def search_and_convert(directory):
    print(f"Start at {directory} Search for model files in the directory...")
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith((".pt", ".pth", ".pkl", ".jit")):
                convert_model_for_cpu(os.path.join(root, file))
    print(f"Completed {directory} Conversion of all model files in the directory!")

# Usage example:
search_and_convert(".")
