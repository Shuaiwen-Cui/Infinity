found = 0;
for (i = 0; i < n && !found; i++)
    for (j = 0; j < m && !found; j++)
        if (a[i] == b[j])
            found = 1;
if (found)
/* got one: a[i-1] == b[j-1] */
... else
    /* didn't find any common element */
    ...