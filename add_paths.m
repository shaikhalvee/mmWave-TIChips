clear all;

ignoreFolders = {'.git', '__MACOSX', '.svn', '.idea', '.vscode', 'private', 'output', 'old_temp'};

currentFile = mfilename('fullpath');
projectRoot = fileparts(currentFile);

folders = genpath(projectRoot);
folders = strsplit(folders, pathsep);

%% Exclude ignoreFolders and empty entries
% remove empty first
folders = folders(~cellfun(@isempty, folders));
% remove ignoreFolders folders
folders = folders(~contains(folders, ignoreFolders));

%% Now add paths safely
addpath(folders{:});
