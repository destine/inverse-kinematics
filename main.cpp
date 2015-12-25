#include "src/root.h"

Root* g_pRoot = NULL;

void myDisplayFunc(void) {
    g_pRoot->render();
}

void myIdleFunc(void) {
    g_pRoot->idle();
}

void myReshapeFunc(int width, int height)
{
    g_pRoot->reshape(width, height);
}

void handleInput(unsigned char key, int x, int y) {
    g_pRoot->handleInput(key, x, y);
    return;
}

int main(int argc, char** argv)
{
    g_pRoot = new Root;

    if (argc != 2)
    {
    	fprintf(stderr, "usage: %s <file>\n", argv[0]);
    	exit(1);
    }

    FILE* input = fopen(argv[1], "r");

    if (!input)
    {
        fprintf(stderr, "error! unable to open file <%s>.\n", argv[1]);
        exit(1);
    }

    g_pRoot->init(argc, argv, input);
    g_pRoot->run(myDisplayFunc, myReshapeFunc, myIdleFunc, handleInput);
    g_pRoot->halt();

    return 0;
}
