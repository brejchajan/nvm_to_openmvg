/**
* @Author: Jan Brejcha <janbrejcha>
* @Email:  brejcha@adobe.com, ibrejcha@fit.vutbr.cz, brejchaja@gmail.com
* @Project: ImmersiveTripReports 2017-2018, Landscape AR 2018
* AdobePatentID="P7840-US"
*
* This code is separate from itr codebase, since it uses
* code under GNU/GPL license. Do NOT merge with itr.
*
* This tool is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation; either
* Version 3 of the License, or (at your option) any later version.
*/

#include "nvmloader.h"

#include "sfm/sfm_data.hpp"
#include "sfm/sfm_data_io.hpp"

using namespace openMVG;
using namespace openMVG::sfm;

int main(int argc, char *argv[])
{
    if (argc == 3)
    {
        const char *nvm_path = argv[1];
        const char *output_path = argv[2];

        try
        {
            //we did not find sfm data, try nvm
            std::cout << "Converting nvm data: " << nvm_path << ", to " << output_path;
            SfM_Data data = NVMLoader::load(nvm_path);
            openMVG::sfm::Save(data, output_path,
                               (ESfM_Data)(VIEWS | EXTRINSICS |
                                INTRINSICS | STRUCTURE));

        }
        catch (std::runtime_error &re)
        {
            std::cerr << "ERROR loading NVM: " << re.what() << std::endl;
            {
                throw std::runtime_error("nvm_to_openmvg: Unable to load sfm_data.");
            }
        }
        return 0;
    }
    return 1;
}
