import h5py
import json
import os
hdf5_path = "robocasa/models/assets/demonstrations_private/2026-03-20-10-57-39_Kitchen/episodes/ep_1773997073_418027/demo.hdf5"
json_path = "robocasa/models/assets/demonstrations_private/2026-03-20-10-57-39_Kitchen/episodes/ep_1773997073_418027/env_info.json"

with open(json_path, "r") as f:
    env_data = f.read() # Citim ca text brut, exact cum scrie în fișier




def get_env_metadata_from_dataset(dataset_path, ds_format="robomimic"):
    dataset_path = os.path.expanduser(dataset_path)
    f = h5py.File(dataset_path, "r")
    if ds_format == "robomimic":
        # Citim atributul crud
        raw_data = f["data"].attrs["env_args"]
        
        # Dacă e deja dicționar (rar), îl folosim direct
        if isinstance(raw_data, dict):
            env_meta = raw_data
        else:
            # Încercăm să-l decodăm dacă e bytes
            if hasattr(raw_data, 'decode'):
                raw_data = raw_data.decode('utf-8')
            
            # Încercăm să-l transformăm în dicționar
            try:
                env_meta = json.loads(raw_data)
                # Dacă rezultatul e tot un string (double encoding), mai dăm un loads
                if isinstance(env_meta, str):
                    env_meta = json.loads(env_meta)
            except:
                print("Eroare la parsarea JSON-ului din HDF5!")
                raise
    else:
        raise ValueError
    f.close()
    return env_meta

get_env_metadata_from_dataset(hdf5_path)