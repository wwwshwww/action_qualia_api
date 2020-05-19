import numpy as np

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
    def __init__(self, super_map: np.ndarray, split_shape: tuple):
        self.pad_value = -2
        self.map_data = self.padding_for_split(super_map, split_shape, self.pad_value)
        self.split_shape = split_shape
        self.resolutions = np.array(self.map_data.shape)//split_shape
        self.resol_par_grid = np.prod(self.resolutions)

        self.density_obstacle = self.density_grid(target=0)
        self.density_road = self.density_grid(target=255)
        self.density_unknown = self.density_grid(target=-1)

    @staticmethod
    def padding_for_split(img, split_shape, pad_value) -> np.ndarray:
        mods = np.array(img.shape)%split_shape
        if all(mods==0):
            return np.array(img)
        elif any(mods==0):
            mods = np.array([mods[i] if not mods[i]==0 else split_shape[i] for i in range(len(split_shape))])

        ## padding at top
        return np.pad(img, tuple((i,0) for i in split_shape-mods), constant_values=(pad_value,pad_value))

    def density_grid(self, target) -> np.ndarray:
        dens = np.zeros(self.split_shape)
        ## for 2D map
        for i in range(len(dens)):
            for j in range(len(dens[0])):
                dens[i,j] = self.resol_par_grid/len(np.where(self.map_data==target)[0])
        return dens

def main():
    img = get_test_img()
    absimg = AbstMap(img, (2,3))
    print(f'{absimg.map_data},\n{absimg.map_data.shape},\n{absimg.resolutions}')

if __name__ == '__main__':
    main()