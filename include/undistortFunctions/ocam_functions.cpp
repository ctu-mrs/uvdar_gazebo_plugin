/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "ocam_functions.h"
#include <cstdio>

//------------------------------------------------------------------------------
int get_ocam_model(struct ocam_model *myocam_model, char *filename) {
  double *pol           = myocam_model->pol;
  double *invpol        = myocam_model->invpol;
  double *xc            = &(myocam_model->xc);
  double *yc            = &(myocam_model->yc);
  double *c             = &(myocam_model->c);
  double *d             = &(myocam_model->d);
  double *e             = &(myocam_model->e);
  int *   width         = &(myocam_model->width);
  int *   height        = &(myocam_model->height);
  int *   length_pol    = &(myocam_model->length_pol);
  int *   length_invpol = &(myocam_model->length_invpol);
  FILE *  f;
  char    buf[CMV_MAX_BUF];
  int     i;

  // Open file
  if (!(f = fopen(filename, "r"))) {
    printf("File %s cannot be opened\n", filename);
    return -1;
  }

  // Read polynomial coefficients
  std::ignore = fgets(buf, CMV_MAX_BUF, f);
  std::ignore = fscanf(f, "\n");
  std::ignore = fscanf(f, "%d", length_pol);
  for (i = 0; i < *length_pol; i++) {
    std::ignore = fscanf(f, " %lf", &pol[i]);
  }

  // Read inverse polynomial coefficients
  std::ignore = fscanf(f, "\n");
  std::ignore = fgets(buf, CMV_MAX_BUF, f);
  std::ignore = fscanf(f, "\n");
  std::ignore = fscanf(f, "%d", length_invpol);
  for (i = 0; i < *length_invpol; i++) {
    std::ignore = fscanf(f, " %lf", &invpol[i]);
  }

  // Read center coordinates
  std::ignore = fscanf(f, "\n");
  std::ignore = fgets(buf, CMV_MAX_BUF, f);
  std::ignore = fscanf(f, "\n");
  std::ignore = fscanf(f, "%lf %lf\n", xc, yc);

  // Read affine coefficients
  std::ignore = fgets(buf, CMV_MAX_BUF, f);
  std::ignore = fscanf(f, "\n");
  std::ignore = fscanf(f, "%lf %lf %lf\n", c, d, e);

  // Read image size
  std::ignore = fgets(buf, CMV_MAX_BUF, f);
  std::ignore = fscanf(f, "\n");
  std::ignore = fscanf(f, "%d %d", height, width);
  printf("WIDTH: %d", *width);
  printf("HEIGHT: %d", *height);

  // Open file
  if (feof(f) || ferror(f)) {
    printf("Could not read all parameters from file %s\n", filename);
    return -1;
  }

  fclose(f);
  return 0;
}

//------------------------------------------------------------------------------
void cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model) {
  double *pol        = myocam_model->pol;
  double  xc         = (myocam_model->xc);
  double  yc         = (myocam_model->yc);
  double  c          = (myocam_model->c);
  double  d          = (myocam_model->d);
  double  e          = (myocam_model->e);
  int     length_pol = (myocam_model->length_pol);
  double  invdet     = 1 / (c - d * e);  // 1/det(A), where A = [c,d;e,1] as in the Matlab file

  double xp = invdet * ((point2D[0] - xc) - d * (point2D[1] - yc));
  double yp = invdet * (-e * (point2D[0] - xc) + c * (point2D[1] - yc));

  double r   = sqrt(xp * xp + yp * yp);  // distance [pixels] of  the point from the image center
  double zp  = pol[0];
  double r_i = 1;
  int    i;

  for (i = 1; i < length_pol; i++) {
    r_i *= r;
    zp += r_i * pol[i];
  }

  // normalize to unit norm
  double invnorm = 1 / sqrt(xp * xp + yp * yp + zp * zp);

  point3D[0] = invnorm * xp;
  point3D[1] = invnorm * yp;
  point3D[2] = invnorm * zp;
}

//------------------------------------------------------------------------------
void world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model) {
  double *invpol        = myocam_model->invpol;
  double  xc            = (myocam_model->xc);
  double  yc            = (myocam_model->yc);
  double  c             = (myocam_model->c);
  double  d             = (myocam_model->d);
  double  e             = (myocam_model->e);
  int     length_invpol = (myocam_model->length_invpol);
  double  norm          = sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1]);
  double  theta         = atan(point3D[2] / norm);
  double  t, t_i;
  double  rho, x, y;
  double  invnorm;
  int     i;

  if (norm != 0) {
    invnorm = 1 / norm;
    t       = theta;
    rho     = invpol[0];
    t_i     = 1;

    for (i = 1; i < length_invpol; i++) {
      t_i *= t;
      rho += t_i * invpol[i];
    }

    x = point3D[0] * invnorm * rho;
    y = point3D[1] * invnorm * rho;

    point2D[0] = x * c + y * d + xc;
    point2D[1] = x * e + y + yc;
  } else {
    point2D[0] = xc;
    point2D[1] = yc;
  }
}
//------------------------------------------------------------------------------
void create_perspecive_undistortion_LUT(cv::Mat *mapx, cv::Mat *mapy, struct ocam_model *ocam_model, float sf) {
  int    i, j;
  int    width     = mapx->cols;  // New width
  int    height    = mapx->rows;  // New height
  float *data_mapx = (float *)mapx->data;
  float *data_mapy = (float *)mapy->data;
  double Nxc       = height / 2.0;
  double Nyc       = width / 2.0;
  double Nz        = -static_cast<double>(width) / sf;
  double M[3];
  double m[2];

  for (i = 0; i < height; i++)
    for (j = 0; j < width; j++) {
      M[0] = (i - Nxc);
      M[1] = (j - Nyc);
      M[2] = Nz;
      world2cam(m, M, ocam_model);
      *(data_mapx + i * width + j) = (float)m[1];
      *(data_mapy + i * width + j) = (float)m[0];
    }
}

//------------------------------------------------------------------------------
void create_panoramic_undistortion_LUT(cv::Mat *mapx, cv::Mat *mapy, float Rmin, float Rmax, float xc, float yc) {
  int    i, j;
  float  theta;
  int    width     = mapx->cols;
  int    height    = mapx->rows;
  float *data_mapx = (float *)mapx->data;
  float *data_mapy = (float *)mapy->data;
  float  rho;

  for (i = 0; i < height; i++)
    for (j = 0; j < width; j++) {
      theta                        = -((float)j) / static_cast<float>(width) * 2 * static_cast<float>(M_PI);  // Note, if you would like to flip the image, just inverte the sign of theta
      rho                          = Rmax - (Rmax - Rmin) / static_cast<float>(height) * static_cast<float>(i);
      *(data_mapx + i * width + j) = yc + rho * sin(theta);  // in OpenCV "x" is the
      *(data_mapy + i * width + j) = xc + rho * cos(theta);
    }
}
