function status = init_rstd_conn(dllPath)
% Wrapper around TI's Init_RSTD_Connection that throws if DLL is missing.
    if ~isfile(dllPath)
        error("RSTD DLL not found: %s", dllPath);
    end
    status = Init_RSTD_Connection(dllPath);
end
