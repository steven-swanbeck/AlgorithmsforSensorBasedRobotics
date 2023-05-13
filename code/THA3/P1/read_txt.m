function [package] = read_txt(filename)
%Accepts a string corresponding to a txt filename to read in. Returns a
%cell array of the relevant values. If the type should return frames of
%data, these will be the contents of the first layer cell array, then the
%data associated with each frame will be nested inside of these. The frame
%data is grouped by type corresponding to the problem specifications, and
%each row corresponds to a single point with x,y,z coordinates as the
%columns.
    testdata = importdata(filename);
    info = testdata.textdata;
    data = testdata.data;

    if size(info) ~= size(data,2)
        var = strsplit(info{1,1}, ', ');
    else
        var = info;
    end

    for i=1:length(var)-1
        thing = str2double(var(i));
        var(i) = num2cell(thing);
    end
    broken = strsplit(var{end}, '-');
    broken2 = strsplit(broken{end}, '.');
    type = broken2{1};

    package = {};
    if type == "calbody"
        ND = var{1};
        NA = var{2};
        NC = var{3};
        package = {data(1:ND, :), data(ND + 1: ND + NA, :), data(ND + 1 + NA:ND + NA + NC, :)};
    elseif type == "calreadings"
        ND = var{1};
        NA = var{2};
        NC = var{3};
        N_frames = var{4};
        bookmark = 0;
        for i = 1:N_frames
            package{i} = {data(bookmark + 1:bookmark + ND, :), data(bookmark + ND + 1: bookmark + ND + NA, :), data(bookmark + ND + 1 + NA:bookmark + ND + NA + NC, :)};
            bookmark = bookmark + ND + NA + NC;
        end
    elseif type == "empivot"
        NG = var{1};
        N_frames = var{2};
        bookmark = 0;
        for i = 1:N_frames
            package{i} = data(bookmark + 1:bookmark + NG, :);
            bookmark = bookmark + NG;
        end
    elseif type == "optpivot"
        ND = var{1};
        NH = var{2};
        N_frames = var{3};
        bookmark = 0;
        for i = 1:N_frames
            package{i} = {data(bookmark + 1:bookmark + ND, :), data(bookmark + ND + 1: bookmark + ND + NH, :)};
            bookmark = bookmark + ND + NH;
        end
    elseif type == "output1"
        NC = var{1};
        N_frames = var{2};
        package{1} = data(1, :);
        package{2} = data(2, :);
        bookmark = 2;
        for i = 1:N_frames
            package{i + 2} = data(bookmark + 1:bookmark + NC, :);
            bookmark = bookmark + NC;
        end
    end
end