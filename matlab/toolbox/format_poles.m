function text = format_poles(poles)
%FORMAT_POLES Compact text for a pole set, e.g. "-9.196 +/- 46.59j, -8.757, 0".
%   Conjugate pairs are listed once.

parts = strings(1, 0);
for p = reshape(poles, 1, [])
    if imag(p) < -1e-9
        continue
    end
    re = real(p);
    if abs(re) < 1e-9
        re = 0;
    end
    if abs(imag(p)) <= 1e-9
        parts(end + 1) = sprintf('%.4g', re); %#ok<AGROW>
    else
        parts(end + 1) = sprintf('%.4g +/- %.4gj', re, imag(p)); %#ok<AGROW>
    end
end
text = join(parts, ", ");
end
