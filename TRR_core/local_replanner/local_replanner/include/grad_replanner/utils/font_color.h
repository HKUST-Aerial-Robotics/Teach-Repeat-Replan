#ifndef _FONT_COLOR_H
#define _FONT_COLOR_H

#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

#define CPRINT1(c, x) cout << c << x << RESET << endl
#define CPRINT2(c, x, y) cout << c << x << y << RESET << endl
#define CPRINT3(c, x, y, z) cout << c << x << y << z << RESET << endl
#define CPRINT4(c, x, y, z, w) cout << c << x << y << z << w << RESET << endl
#define CPRINT5(c, x, y, z, w, a) cout << c << x << y << z << w << a << RESET << endl
#define CPRINT6(c, x, y, z, w, a, b) cout << c << x << y << z << w << a << b << RESET << endl
#define SETC(c) cout << c 
#define REC RESET << endl
#define SETR SETC(BOLDRED)
#define SETG SETC(BOLDGREEN)
#define SETB SETC(BOLDBLUE)
#define SETY SETC(BOLDYELLOW)
#define SETM SETC(BOLDMAGENTA)
#define SETCY SETC(BOLDCYAN)

#endif