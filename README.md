# gliderstudio


## Classes

### MyBatch
Parse through a folder with tracking data (.csv files).

Folder structure: `parentfolder` (where code is) > datafolder > sessionfolder > csv files.

Inputs:
* `sessionfolder` : folder inside datafolder (string)
* `datafolder` : default "trackingData/"

```
% create object to use data methods
mysession = MyBatch;
mysession = mysession.newBatch("trackingData/", sessionfolder);
```


Relevant outputs:

* `folderpath`: path (relative to current folder) to the data files
* `filename`: name of csv file inside sessionfolder.

```
% get folderpath
folderpath = mysession.folderpath;
%  get name of k-th file within session folder
filename = mysession.getfname(k);
```

### MyTake

Import data from a single csv file; run checks to get rid of gaps (when all markers were occluded) and 
extra frames (after glider hit the floor or left MoCap arena). Access time and 6DoF rigid body pose in global coordinates 
wrt Motion Studio global frame of reference.
> GLOBAL frame of reference: 
> ORIGIN ~ middle of MoCap arena; ORIENTATION: Y-up, X positive towards entrance door

Inputs: 

* folderpath
* filename

Relevant outputs:
* `time`: array with time stamps of data collection
* `position`: XYZ body position wrt to global coordinates, in mm
* `rotation`: XYZ rotation wrt to global coordinates, between -180 - 180 degrees

```
    % new take object
    take = MyTake;
    % collect data and runs checks
    take = take.new(fname, fpath);

    % get data for analysis
    [time, position, rotation] = take.getData;
```

> Note: no changes in data (no interpolation, change of coordinates, angle etc). 

Plots:
* 3D data points in global frame, in meters for better visualization
* Rotation angles (also wrt global frame), between -180 - 180 degrees 

```
    % plot tracking data in global coordinates
    plotfoldername = "takePlots";
    take.plotData(plotfoldername);
```
* Figure
* * title: name of rigid body
* * subtitle: take name
* Save as
* * .png file 
* * in chosen `plotfoldername` (located inside parentfolder)
* * file name: take name

## Author
Beatriz Asfora, PhD.

Motion Studio, Cornell University
