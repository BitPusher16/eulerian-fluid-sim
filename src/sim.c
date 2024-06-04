#include <stdlib.h>
#include <stdio.h>
#include <raylib.h>
#include <math.h>

// fluid sim.
// modeled after example by Matthias Muller.
// https://www.youtube.com/watch?v=iKAVRgIrUOU

void include_check();
int** calloc_2d_int(int rows, int cols);
float** calloc_2d_float(int rows, int cols);
void free_2d_int(int**, int rows);
void free_2d_float(float**, int rows);

float divergence(float** u, float** v, int i, int j);
void flow_direction(float** u, float** v, int i, int j, float* u_out, float* v_out);
Color float_to_gray(float x);
Color float_to_ryg(float x);
float logistic(float x);
float logistic_neg_one_to_one(float x);
float minf(float a, float b);
float maxf(float a, float b);
float boundf(float x, float a, float b);
int mini(int a, int b);
int maxi(int a, int b);
int boundi(int x, int a, int b);

float x_m_y_m_to_weighted_u(
    float** u, int grid_rows, int grid_cols, float h,
    float xpos_m, float ypos_m
);

float x_m_y_m_to_weighted_v(
    float** v, int grid_rows, int grid_cols, float h,
    float xpos_m, float ypos_m
);

int main(int argc, char** argv){

    if(argc < 2){
        printf("%d\n", argc);
        printf("usage:\n");
        printf("sim /path/to/raster.png\n");
        return 0;
    }

    // todo: read grid dimensions from raster.
    // update: no, set grid dims independently. not all sims will use raster.
    //const char* fname = getenv("RASTER_PATH");
    const char* fname = argv[1];
    printf("%s\n", fname);
    Image image = LoadImage(fname);
    Color* image_colors = LoadImageColors(image);
    int raster_w_px = image.width;
    int raster_h_px = image.height;

    // simulation params

    int grid_w_m = 16;
    int grid_h_m = 12;
    int cells_per_m = 10;
    float h = 1.0 / (float)cells_per_m; // meters per cell. width cell edge.
    //int cell_width_cm = 250; // h
    int px_per_cell = 4;

    float flow_speed_mps = 4.0;
    int target_fps = 30;
    int sim_iters = 20;
    float relaxation = 1.9;

    int grid_cols = grid_w_m * cells_per_m;
    int grid_rows = grid_h_m * cells_per_m;
    //int grid_cols = raster_w_px;
    //int grid_rows = raster_h_px;
    int screen_w_px = grid_cols * px_per_cell;
    int screen_h_px = grid_rows * px_per_cell;

    enum render_modes{
        WALLS,
        DIVERGENCE,
        PRESSURE,
        FLOW_DIRECTION,
        SMOKE
    };
    enum render_modes render_mode = FLOW_DIRECTION;

    // simulation data

    // grid is stored as a 2d array m rows by n columns.
    // i indexes rows and increases down.
    // j indexes columns and increases right.
    // note this does not follow the convention adopted by Matthias.

    int** s = calloc_2d_int(grid_rows, grid_cols);
    float** p = calloc_2d_float(grid_rows, grid_cols);
    float** m = calloc_2d_float(grid_rows, grid_cols);
    float** new_m = calloc_2d_float(grid_rows, grid_cols);

    float** u = calloc_2d_float(grid_rows+1, grid_cols+1);
    float** v = calloc_2d_float(grid_rows+1, grid_cols+1);
    float** new_u = calloc_2d_float(grid_rows+1, grid_cols+1);
    float** new_v = calloc_2d_float(grid_rows+1, grid_cols+1);

    // the wall velocities of cell i,j are available at:
    // u[i][j] (fluid entering from left wall)
    // v[i][j] (fluid entering from top wall)
    // u[i][j+1] (fluid exiting from right wall)
    // v[i+1][j] (fluid exiting from bottom wall)


    // set cell states according to existing obstacles.
    // 1 means cell holds fluid, 0 means cell is an obstacle.
    // todo: rather than if statements, implement as array of obstacles,
    // one for each wall, and one for obstacle read in from binary img.
    for(int i = 0; i < grid_rows; i++){
        for(int j = 0; j < grid_cols; j++){
            int fill = 1;
            // top row
            if(i == 0){ fill = 0;}
            // bottom row
            else if(i == grid_rows - 1){ fill = 0;}
            // left wall
            else if(j == 0){ fill = 0; }

            // right wall (note Matthias leaves right wall open)
            else if(j == grid_cols - 1){ fill = 0; }

            // barrier box
            //else if(i > 3 && i < 8 && j > 3 && j < 8){ fill = 0;}
            // raster
            else if( image_colors[i * grid_cols + j ].r == 0) {fill = 0;}
            s[i][j] = fill;
        }
    }

    // set velocities for left wall.
    // note we are setting the right wall of the leftmost cells to positive val.
    for(int i = 1; i < grid_rows-1; i++){
        u[i][1] = flow_speed_mps;

        // debug
        //u[i][2] = flow_speed_mps;
    }

    // set velocities for right wall. (note Matthias does not set vel for right)
    // note we are setting the left wall of the rightmost cells to positive val.
    for(int i = 1; i < grid_rows-1; i++){
        u[i][grid_cols-1] = flow_speed_mps;
    }


    // simulation loop:
    //   make incompressible.
    //   advect velocities.
    //   (draw to screen.)

    InitWindow(screen_w_px, screen_h_px, "eulerian fluid sim");
    SetTargetFPS(target_fps);

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        if(dt <= 0.0){dt = 0.000001;}

        // update

        // place new smoke particles.
        // todo: do this properly, create a grid that tracks location of smoke emitters
        for(int i = 1; i < grid_rows-1; i++){
            if( (i / 4) % 4 == 3){m[i][1] = 1.0;}
        }


        // satisfy incompressibility constraint with Gauss-Seidel.
        for(int iter = 0; iter < sim_iters; iter++){

            // we will not update the divergence of the outermost cells.
            // this allows us to skip some checks when updating divergence.
            // note this will cause us to skip some fluid cells (right edge).
            // note we are still skipping some inner cells (obstacles).

            for(int i = 1; i < grid_rows-1; i++){
                for(int j = 1; j < grid_cols-1; j++){
                    if(s[i][j] == 0){continue;} // skip obstacles.
                    float d = relaxation * divergence(u, v, i, j);

                    // neighbor states.
                    float ns = (float)(s[i-1][j] + s[i+1][j] + s[i][j-1] + s[i][j+1]);

                    // under normal conditions, we don't need to check 
                    // state of neigboring cell before updating edge flow.
                    // however, if advection goes poorly, 
                    // inf may get into some cells, giving d=inf,
                    // and circumventing multiplication by 0.
                    // to be safe, we check the other side of each
                    // wall we are about to update.
                    if(s[i][j-1]!=0){ u[i][j]   += d * ((float)s[i][j-1]/ns); }
                    if(s[i][j+1]!=0){ u[i][j+1] -= d * ((float)s[i][j+1]/ns); }

                    if(s[i-1][j]!=0){ v[i][j]   -= d * ((float)s[i-1][j]/ns); }
                    if(s[i+1][j]!=0){ v[i+1][j] += d * ((float)s[i+1][j]/ns); }
                }
            }
        }

        // advect velocity fields.
        // here we advect velocities for inner (non-obstacle) walls.
        // note: walls, not cells.
        // in first pass, compute new values for each u, v.
        // in second pass, overwrite existing values with new values.
        // take care not to overwrite sources and sinks.

        for(int i = 1; i < grid_rows-1; i++){
            for(int j = 1; j < grid_cols-1; j++){
                if(s[i][j] == 0){continue;}

                if(s[i][j-1] != 0){
                    // interpolate a value for v at this u.
                    float v_at_u = (v[i][j-1]+v[i][j]+v[i+1][j-1]+v[i+1][j])/4.0;

                    // find the x,y of this u in meters.
                    float u_xpos_m = (float)j * h;
                    float u_ypos_m = (((float)i)+0.5) * h;

                    // subtract u,v (adjusted for time) from the u,v at this u.
                    float u_src_xpos_m = u_xpos_m - dt*u[i][j];
                    float u_src_ypos_m = u_ypos_m - dt*v_at_u;

                    // validate the velocity source location.
                    // if the source location falls outside the grid
                    // or lands on an obstacle, do not update.
                    //int u_src_i = floor(u_src_xpos_m / h);
                    //int u_src_j = floor(u_src_ypos_m / h);
                    //if(
                    //    u_src_i < 0 || u_src_i > grid_rows-1
                    //    || u_src_j < 0 || u_src_j > grid_cols-1
                    //    || s[u_src_i][u_src_j] == 0
                    //){new_u[i][j] = u[i][j]; continue;}

                    // interpolate a u at our source x,y and write this to new_u.
                    float u_src = x_m_y_m_to_weighted_u(u, grid_rows, grid_cols, h, u_src_xpos_m, u_src_ypos_m);
                    new_u[i][j] = u_src;
                }

                if(s[i-1][j] != 0){
                    float u_at_v = (u[i-1][j]+u[i-1][j+1]+u[i][j]+u[i][j+1])/4.0;

                    float v_xpos_m = (((float)j)+0.5) * h;
                    float v_ypos_m = (float)i * h;

                    float v_src_xpos_m = v_xpos_m - dt*v[i][j];
                    float v_src_ypos_m = v_ypos_m - dt*u_at_v;

                    // validate the velocity source location.
                    // if the source location falls outside the grid
                    // or lands on an obstacle, do not update.
                    //int v_src_i = floor(v_src_xpos_m / h);
                    //int v_src_j = floor(v_src_ypos_m / h);
                    //if(
                    //    v_src_i < 0 || v_src_i > grid_rows-1
                    //    || v_src_j < 0 || v_src_j > grid_cols-1
                    //    || s[v_src_i][v_src_j] == 0
                    //){new_v[i][j] = v[i][j]; continue;}

                    float v_src = x_m_y_m_to_weighted_v(v, grid_rows, grid_cols, h, v_src_xpos_m, v_src_ypos_m);
                    new_v[i][j] = v_src;
                }
            }
        }
        for(int i = 1; i < grid_rows-1; i++){
            for(int j = 1; j < grid_cols-1; j++){
                if(s[i][j] == 0){continue;}
                if(s[i][j-1] != 0){
                    u[i][j] = new_u[i][j];
                }
                if(s[i-1][j] != 0){
                    v[i][j] = new_v[i][j];
                }
            }
        }

        // advect smoke.
        // todo: update the advected velocities after calculating new smoke vals.
        for(int i = 1; i < grid_rows-1; i++){
            for(int j = 1; j < grid_cols-1; j++){
                if(s[i][j] == 0){continue;}

                float cell_u = (u[i][j] + u[i][j+1]) / 2.0;
                float cell_v = (v[i][j] + v[i+1][j]) / 2.0;
                //float cell_xpos_m = i * h + 0.5;
                //float cell_ypos_m = j * h + 0.5;
            }
        }



        // draw
        BeginDrawing();
        ClearBackground(RAYWHITE);

        for(int i = 0; i < grid_rows; i++){
            for(int j = 0; j < grid_cols; j++){
                Color color = WHITE;
                float debug_divergence = 0.0;

                if(render_mode == WALLS){
                    if(s[i][j] == 0){ color = BLACK;}
                    else{ color = ((i+j) % 2 == 0) ? WHITE : LIGHTGRAY; }
                }
                else if(render_mode == SMOKE){
                    float val = m[i][j];
                    color = (Color){val*255, val*255, val*255, 255};

                }
                else if(render_mode == DIVERGENCE){
                    float d = divergence(u, v, i, j);
                    debug_divergence = d;
                    int tmp = logistic(d) * 255;
                    color = (Color){tmp, tmp, tmp, 255};

                    //float d = divergence(u, v, i, j);
                    //color = float_to_ryg(d);
                    //color = float_to_gray(d);
                }
                else if(render_mode == FLOW_DIRECTION){
                    if(s[i][j] == 0){color = BLACK;}
                    else{
                        float fu, fv;
                        flow_direction(u, v, i, j, &fu, &fv);
                        //color = vec_to_rygcbv(fu, fv);
                        //color = float_to_gray(fv);
                        //float angle = atan2f(fu, fv) * (180.0/M_PI);

                        float theta_rad = atan2(fu, fv);
                        int theta_deg =floor( (theta_rad/M_PI*180) + (theta_rad > 0 ? 0 : 360));
                        color = ColorFromHSV(theta_deg, 1.0,0.7);
                    }
                }

                DrawRectangle(
                    j * px_per_cell, i * px_per_cell,
                    px_per_cell, px_per_cell, color
                );

                //char buff[100];
                //snprintf(buff, sizeof(buff), "%f", debug_divergence);
                //DrawText(buff, j*px_per_cell, i*px_per_cell, 8, BLUE);
            }
        }
        EndDrawing();
    }
    CloseWindow();

    free_2d_int(s, grid_rows);
    free_2d_float(p, grid_rows);
    free_2d_float(m, grid_rows);
    free_2d_float(new_m, grid_rows);

    free_2d_float(u, grid_rows);
    free_2d_float(v, grid_rows);
    free_2d_float(new_u, grid_rows);
    free_2d_float(new_v, grid_rows);

    UnloadImageColors(image_colors);
    UnloadImage(image);

    return 0;

}

int** calloc_2d_int(int rows, int cols){
    int** ret = (int**)malloc(rows * sizeof(int*));
    for(int i = 0; i < rows; i++){
        //ret[i] = (int*)malloc(cols * sizeof(int));
        ret[i] = (int*)calloc(cols, sizeof(int));
    }
    return ret;
}

float** calloc_2d_float(int rows, int cols){
    float** ret = (float**)malloc(rows * sizeof(float*));
    for(int i = 0; i < rows; i++){
        //ret[i] = (float*)malloc(cols * sizeof(float));
        ret[i] = (float*)calloc(cols, sizeof(float));
    }
    return ret;
}

void free_2d_int(int** arr, int rows){
    for(int i = 0; i < rows; i++){ free(arr[i]); }
    free(arr);
}

void free_2d_float(float** arr, int rows){
    for(int i = 0; i < rows; i++){ free(arr[i]); }
    free(arr);
}

float divergence(float** u, float** v, int i, int j){
    float d = 0.0;
    d -= u[i][j];
    d += u[i][j+1];
    d += v[i][j];
    d -= v[i+1][j];

    return d;
}

void flow_direction(float** u, float** v, int i, int j, float* u_out, float* v_out){
    // compute the average flow in the u direction.
    // compute average flow in the v direction.
    *u_out = (u[i][j] + u[i][j+1])/2.0;
    *v_out = (v[i][j] + v[i+1][j])/2.0;
}

float logistic_neg_one_to_one(float x){
    return -1.0 + (2.0 / (1.0 + exp(-x)));
}

float logistic(float x){
    return 1.0 / (1.0 + exp(-x));
}

Color float_to_gray(float x){
    // Zero mapped to gray, negative to black, positive to white.
    float v = logistic(x) * 255;
    return (Color){v, v, v, 255};
}

Color float_to_ryg(float x){
    // Zero mapped to yellow, negative to red, positive to green.
    int r = 255;
    int g = 255;
    int b = 0;
    float offset = logistic_neg_one_to_one(x);
    if(offset > 0){r = 0;}
    if(offset < 0){g = 0;}
    return (Color){r, g, b, 255};
}

float minf(float a, float b){ 
    if(a < b){return a;}
    else{return b;}
}

float maxf(float a, float b){
    if(a > b){return a;}
    else{return b;}
}

float boundf(float x, float a, float b){
    return minf(maxf(x, a), b);
}

int mini(int a, int b){
    if(a < b){return a;}
    else{return b;}
}

int maxi(int a, int b){
    if(a > b){return a;}
    else{return b;}
}

int boundi(int x, int a, int b){
    return mini(maxi(x, a), b);
}


float x_m_y_m_to_weighted_u(
    float** u, int grid_rows, int grid_cols, float h,
    float xpos_m, float ypos_m
){
    // find tl_u_i,tl_u_j for the next u to the top-left of our x,y.
    // find tl_u_x, tl_u_y in meters for this u.

    xpos_m = boundf(xpos_m, 0.0, grid_cols * h);
    ypos_m = boundf(ypos_m, 0.0, grid_rows * h);

    float xpos_cells = xpos_m / h;
    float ypos_cells = ypos_m / h;

    // if x,y falls in top half of cell, we need next cell up
    if(fmod(ypos_cells, 1) < 0.5){ypos_cells -= 1;}

    int tl_u_i = floor(ypos_cells);
    int tl_u_j = floor(xpos_cells);


    tl_u_i = boundi(tl_u_i, 0, grid_rows-1);
    tl_u_j = boundi(tl_u_j, 0, grid_cols-1);

    float tl_u_x_m = (float)tl_u_j * h;
    float tl_u_y_m = (float)tl_u_i * h + (0.5*h);

    xpos_m = boundf(xpos_m, tl_u_x_m, tl_u_x_m + h);
    ypos_m = boundf(ypos_m, tl_u_y_m, tl_u_y_m + h);

    // bilinear interpolation.
    // https://en.wikipedia.org/wiki/Bilinear_interpolation
    // "The product of the value at the desired point 
    // and the entire area is equal to the sum of the 
    // products of the value at each corner and the 
    // partial area diagonally opposite the corner (corresponding colours)."

    float a = ypos_m - tl_u_y_m;
    float b = xpos_m - tl_u_x_m;
    float c = h - a;
    float d = h - b;

    //float sum = 0.0
    //    + u[tl_u_i  ][tl_u_j  ] * c * d
    //    + u[tl_u_i  ][tl_u_j+1] * b * c
    //    + u[tl_u_i+1][tl_u_j  ] * a * d
    //    + u[tl_u_i+1][tl_u_j+1] * a * b
    //;
    //return sum / (h*h);

    float sum = 0.0
        + u[tl_u_i  ][tl_u_j  ]
        + u[tl_u_i  ][tl_u_j+1]
        + u[tl_u_i+1][tl_u_j  ]
        + u[tl_u_i+1][tl_u_j+1]
    ;
    return sum / 4.0;

    //return u[tl_u_i][tl_u_j];
}

float x_m_y_m_to_weighted_v(
    float** v, int grid_rows, int grid_cols, float h,
    float xpos_m, float ypos_m
){
    // find tl_v_i,tl_v_j for the next v to the top-left of our x,y.
    // find tl_v_x, tl_v_y in meters for this v.

    xpos_m = boundf(xpos_m, 0.0, grid_cols * h);
    ypos_m = boundf(ypos_m, 0.0, grid_rows * h);

    float xpos_cells = xpos_m / h;
    float ypos_cells = ypos_m / h;

    // if x,y falls in left half of cell, we need next cell left
    if(fmod(xpos_cells, 1) < 0.5){xpos_cells -= 1;}

    int tl_v_i = floor(ypos_cells);
    int tl_v_j = floor(xpos_cells);

    tl_v_i = boundi(tl_v_i, 0, grid_rows-1);
    tl_v_j = boundi(tl_v_j, 0, grid_cols-1);

    float tl_v_x_m = (float)tl_v_j * h + (0.5*h);
    float tl_v_y_m = (float)tl_v_i * h;

    xpos_m = boundf(xpos_m, tl_v_x_m, tl_v_x_m + h);
    ypos_m = boundf(ypos_m, tl_v_y_m, tl_v_y_m + h);

    // bilinear interpolation.
    // https://en.wikipedia.org/wiki/Bilinear_interpolation
    // "The product of the value at the desired point 
    // and the entire area is equal to the sum of the 
    // products of the value at each corner and the 
    // partial area diagonally opposite the corner (corresponding colours)."

    float a = ypos_m - tl_v_y_m;
    float b = xpos_m - tl_v_x_m;
    float c = h - a;
    float d = h - b;

    //float sum = 0.0
    //    + v[tl_v_i  ][tl_v_j  ] * c * d
    //    + v[tl_v_i  ][tl_v_j+1] * b * c
    //    + v[tl_v_i+1][tl_v_j  ] * a * d
    //    + v[tl_v_i+1][tl_v_j+1] * a * b
    //;
    //return sum / (h*h);

    float sum = 0.0
        + v[tl_v_i  ][tl_v_j  ]
        + v[tl_v_i  ][tl_v_j+1]
        + v[tl_v_i+1][tl_v_j  ]
        + v[tl_v_i+1][tl_v_j+1]
    ;
    return sum / 4.0;

    //return v[tl_v_i][tl_v_j];

}

int times_three(int x){
    int ret = x * 3;
    return ret;
}



















