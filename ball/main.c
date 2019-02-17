#include "darknet.h"

void test_ball(char *cfgfile, char *weightfile, char *filename);

int main(){
    char cfg[100], weights[100], filename[100];
    printf("usage: [cfg] [weights] [filename]\n");
    scanf("%s%s%s", &cfg, &weights, &filename);
    if(!cfg || !weights || !filename){
        printf("usage: [cfg] [weights] [filename]\n");
        return 1;
    }
    test_ball(cfg, weights, filename);

    return 0;
}
/*
void run_ball(int argc, char **argv)
{
    if (argc < 4 || strcmp(argv[2], "test"))
    {
        fprintf(stderr, "usage: %s %s test [cfg] [weights (optional)]\n", argv[0], argv[1]);
        return;
    }

    char *cfg = argv[3];
    char *weights = (argc > 4) ? argv[4] : 0;
    char *filename = (argc > 5) ? argv[5] : 0;

    test_writing(cfg, weights, filename);
}
*/
void test_ball(char *cfgfile, char *weightfile, char *filename)
{
    network *net = parse_network_cfg(cfgfile);
    if (weightfile)
    {
        load_weights(net, weightfile);
    }
    set_batch_network(net, 1);
    srand(2222222);
    clock_t time;
    char buff[256];
    char *input = buff;
    while (1)
    {
        if (filename)
        {
            strncpy(input, filename, 256);
        }
        else
        {
            printf("Enter Image Path: ");
            fflush(stdout);
            input = fgets(input, 256, stdin);
            if (!input)
                return;
            strtok(input, "\n");
        }

        image im = load_image_color(input, 0, 0);
        resize_network(net, im.w, im.h);
        printf("%d %d %d\n", im.h, im.w, im.c);
        float *X = im.data;
        time = clock();
        network_predict(net, X);
        printf("%s: Predicted in %f seconds.\n", input, sec(clock() - time));
        image pred = get_network_image(net);

        image upsampled = resize_image(pred, im.w, im.h);
        image thresh = threshold_image(upsampled, .5);
        pred = thresh;

        show_image(pred, "prediction", 1000);
        show_image(im, "orig", 1000);
#ifdef OPENCV
        cvWaitKey(0);
        cvDestroyAllWindows();
#endif

        free_image(upsampled);
        free_image(thresh);
        free_image(im);
        if (filename)
            break;
    }
}