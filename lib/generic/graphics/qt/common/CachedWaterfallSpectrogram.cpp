/**
 * \file lib/generic/graphics/qt/common/CachedWaterfallSpectrogram.cpp
 * \version 1.0
 *
 * \section COPYRIGHT
 *
 * Copyright 2012-2013 The Iris Project Developers. See the
 * COPYRIGHT file at the top-level directory of this distribution
 * and at http://www.softwareradiosystems.com/iris/copyright.html.
 *
 * \section LICENSE
 *
 * This file is part of the Iris Project.
 *
 * Iris is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * Iris is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * A copy of the GNU Lesser General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 * \section DESCRIPTION
 *
 * Reimplementation of a QwtPlotSpectrogram that caches the rendered
 * spectrogram image for faster rendering of a waterfall display.
 */

#include "CachedWaterfallSpectrogram.h"
#include <QPainter>
#include <qwt_scale_map.h>

CachedWaterfallSpectrogram::CachedWaterfallSpectrogram(const QString &title)
  :QwtPlotSpectrogram(title)
  ,numRows_(0)
  ,newRows_(0)
{}

CachedWaterfallSpectrogram::~CachedWaterfallSpectrogram()
{}

QImage CachedWaterfallSpectrogram::renderImage(const QwtScaleMap &xMap,
                                const QwtScaleMap &yMap,
                                const QRectF &area,
                                const QSize &imageSize) const
{
  if(cacheArea_ == area &&
     cacheImageSize_ == imageSize)
  {
    QRectF rect = QwtScaleMap::transform(xMap, yMap, area);
/*
    // There will be a mismatch between the data resolution and the
    // image resolution in pixels. We calculate a number of rows
    // that is within a tolerance of an even number of pixels and
    // only shift by multiples of that.
    if(newRows_ < rowSetSize_)
      return cacheImage_;
    int shiftRows = rowSetSize_*(newRows_/rowSetSize_);
    newRows_ -= shiftRows;
*/
    int shiftRows = newRows_;
    int tileImageHeight = round((float)shiftRows * imageSize.height() / shiftRows);
    int tileAreaHeight = round((float)shiftRows * area.height() / shiftRows);
    if(tileImageHeight < 1)
      return cacheImage_;

    QRectF tile( 0, 0, area.width(), tileAreaHeight);
    QSize tileSize(imageSize.width(), tileImageHeight);
    QImage tileImage = QwtPlotSpectrogram::renderImage(xMap, yMap, tile, tileSize);

    QRectF cacheSrc(0, 0, imageSize.width(), imageSize.height()-tileImageHeight);
    QRectF cacheDst(0, tileImageHeight, imageSize.width(), imageSize.height()-tileImageHeight);
    QRectF tileRect(0, 0, imageSize.width(), tileImageHeight);

    QImage newImage(imageSize, cacheImage_.format());
    QPainter painter(&newImage);
    newImage.fill(0);
    painter.drawImage(tileRect, tileImage, tileRect);
    painter.drawImage(cacheDst, cacheImage_, cacheSrc);
    cacheImage_ = newImage;
    cacheArea_ = area;
    cacheImageSize_ = imageSize;
  }else{
    //pixPerRow_ = (float)imageSize.height() / numRows_;
    //calcRowSetSize();
    cacheArea_ = area;
    cacheImageSize_ = imageSize;
    cacheImage_ = QwtPlotSpectrogram::renderImage(xMap, yMap, area, imageSize);
    newRows_ = 0;
  }
  return cacheImage_;
}

/** Calculate the number of data rows that gives us an almost even number
 * of pixels to shift.
 */
void CachedWaterfallSpectrogram::calcRowSetSize() const
{
  float rem = 1;
  float tol = 0.2;
  int count=0;
  while(rem > tol)
  {
    float cur = ++count*pixPerRow_;
    rem = (cur - (int)cur)/cur;
  }
  rowSetSize_ = count;
}

void CachedWaterfallSpectrogram::setNumRows(int nRows)
{
  numRows_ = nRows;
}

void CachedWaterfallSpectrogram::newRowAdded()
{
  newRows_++;
}

