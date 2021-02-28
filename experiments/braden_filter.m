function [y_e, m, y_min, y_max] = braden_filter(y, m, m_prev, y_min, y_max)

if m_prev <= 0 && m > 0
    y_min = y;
elseif m_prev >= 0 && m < 0
    y_max = y;
end

y_e = mean([y_min y_max]);

end