import numpy as np

from typing import Tuple

def get_test_img():
    im = np.zeros([500, 500], dtype=int)
    im[2,2:10] = 255
    im[4,4] = 255
    im[4,35] = 255
    im[42,7] = 255
    im[48, 40] = 255
    im[25,1] = 255
    im[22,26] = 255
    im[23,25] = 255
    im[24,24] = 255
    im[29,29] = 255

    im[455,455] = 255
    im[451,456] = 255
    return im

class AbstMap():
    def __init__(self, super_map: np.ndarray, split_shape):
        self.pad_value = -2
        self.pad_widths, self.map_data = self.padding_for_split(super_map, split_shape, self.pad_value)
        self.split_shape = split_shape
        self.resolutions = np.array(self.map_data.shape)//split_shape
        self.resol_par_mesh = np.prod(self.resolutions)

        self.density_obstacle = self.density_mesh(target=0)
        self.density_road = self.density_mesh(target=255)
        self.density_unknown = self.density_mesh(target=-1)

    @staticmethod
    def padding_for_split(img, split_shape, pad_value) -> (tuple, np.ndarray):
        mods = np.array(img.shape)%split_shape
        if all(mods==0):
            return tuple(0 for _ in range(len(split_shape))), np.array(img)
        elif any(mods==0):
            mods = np.array([mods[i] if not mods[i]==0 else split_shape[i] for i in range(len(split_shape))])
        pad_widths = tuple(split_shape-mods)
        ## padding at top
        return pad_widths, np.pad(img, tuple((i,0) for i in pad_widths), constant_values=(pad_value,pad_value))

    def get_abst_index(self, super_index):
        return (np.array(super_index)+self.pad_widths)//self.resolutions

    def trim(self, index: Tuple[int]):
        trim_slice = tuple(slice(n*self.resolutions[i], (n+1)*self.resolutions[i]) for i,n in enumerate(index))
        return self.map_data[trim_slice]

    def _df(self, arr, func: callable, kwargs=None, index_list=None):
        if index_list is None:
            index_list = []
        if len(index_list)==len(self.split_shape):
            ti = tuple(index_list)
            t = self.trim(ti)
            arr[ti] = func(t, **kwargs)
            return
        
        for i in range(self.split_shape[len(index_list)]):
            ti = list(index_list)
            ti.append(i)
            self._df(arr, func, kwargs, ti)
        
    def _density(self, arr: np.ndarray, val) -> float:
        return len(np.where(arr==val)[0])/self.resol_par_mesh

    def density_mesh(self, target) -> np.ndarray:
        dens = np.zeros(self.split_shape)
        self._df(dens, self._density, {'val':target})

        ## for 2D map
        # for i in range(len(dens)):
        #     for j in range(len(dens[0])):
        #         t = self.trim((i,j))
        #         dens[i,j] = len(np.where(t==target)[0])/self.resol_par_mesh

        return dens

def main():
    img = get_test_img()
    abst = AbstMap(img, (2,3))
    print(f'{abst.map_data},\n{abst.map_data.shape},\n{abst.resolutions}')
    print(f'obs: \n{abst.density_obstacle}, \nroad: \n{abst.density_road}')
    print(f'get abst index from (499,499): {abst.get_abst_index((499,499))}')

if __name__ == '__main__':
    main()