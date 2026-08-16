#import "MLNDisplayUtils.h"
#import "MLNFoundation_Private.h"
#import "MLNLoggingConfiguration_Private.h"
#import "MLNMapView+Metal.h"

#import <mbgl/mtl/renderable_resource.hpp>

#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import <QuartzCore/CAMetalLayer.h>

#import <Metal/Metal.hpp>

// ─── Watermark Isomaps : COMPILÉ UNIQUEMENT pour le build SDK vendu ───────────────
// Flag ISOMAPS_SDK_WATERMARK (bazel : --define=isomaps_watermark=on). Build par défaut
// / app Isomaps → aucun watermark.
#if defined(ISOMAPS_SDK_WATERMARK)
#import <simd/simd.h>
#include <algorithm>
#import "isomaps_watermark_data.h"

// ─── ISOMAPS WATERMARK (attribution incrustée dans le rendu Metal — NON RETIRABLE) ───
// Le logo Isomaps est embarqué dans le binaire (kIsomapsWatermarkRGBA) et dessiné à
// chaque frame sur le drawable, juste avant présentation, en bas à droite. Aucune API
// publique ne permet de le masquer ; le retirer exige de modifier ET recompiler le SDK.
@interface MLNIsomapsWatermark : NSObject
+ (void)drawInto:(id<MTLCommandBuffer>)cmd
        drawable:(id<CAMetalDrawable>)drawable
          device:(id<MTLDevice>)device;
@end

@implementation MLNIsomapsWatermark {
}

static id<MTLRenderPipelineState> sWmPipeline = nil;
static id<MTLTexture> sWmTexture = nil;
static id<MTLSamplerState> sWmSampler = nil;

+ (void)ensure:(id<MTLDevice>)device pixelFormat:(MTLPixelFormat)pf {
  if (sWmPipeline && sWmTexture && sWmSampler) return;

  NSString* src =
      @"#include <metal_stdlib>\n"
      @"using namespace metal;\n"
      @"struct VOut { float4 pos [[position]]; float2 uv; };\n"
      @"vertex VOut wm_v(uint vid [[vertex_id]], constant float4* rect [[buffer(0)]]) {\n"
      @"  float2 c[4] = { float2(0,0), float2(1,0), float2(0,1), float2(1,1) };\n"
      @"  float2 q = c[vid];\n"
      @"  float2 p = rect[0].xy + q * rect[0].zw;\n"       // p in [0,1], y up
      @"  VOut o; o.pos = float4(p.x*2.0-1.0, p.y*2.0-1.0, 0.0, 1.0);\n"
      @"  o.uv = float2(q.x, 1.0-q.y); return o;\n"
      @"}\n"
      @"fragment float4 wm_f(VOut in [[stage_in]], texture2d<float> t [[texture(0)]],\n"
      @"                     sampler s [[sampler(0)]]) { return t.sample(s, in.uv); }\n";

  NSError* err = nil;
  id<MTLLibrary> lib = [device newLibraryWithSource:src options:nil error:&err];
  if (!lib) return;
  MTLRenderPipelineDescriptor* pd = [[MTLRenderPipelineDescriptor alloc] init];
  pd.vertexFunction = [lib newFunctionWithName:@"wm_v"];
  pd.fragmentFunction = [lib newFunctionWithName:@"wm_f"];
  pd.colorAttachments[0].pixelFormat = pf;
  pd.colorAttachments[0].blendingEnabled = YES;
  pd.colorAttachments[0].rgbBlendOperation = MTLBlendOperationAdd;
  pd.colorAttachments[0].alphaBlendOperation = MTLBlendOperationAdd;
  pd.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
  pd.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorSourceAlpha;
  pd.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
  pd.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
  sWmPipeline = [device newRenderPipelineStateWithDescriptor:pd error:&err];
  if (!sWmPipeline) return;

  MTLTextureDescriptor* td =
      [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                         width:kIsomapsWatermarkW
                                                        height:kIsomapsWatermarkH
                                                     mipmapped:NO];
  td.usage = MTLTextureUsageShaderRead;
  sWmTexture = [device newTextureWithDescriptor:td];
  [sWmTexture replaceRegion:MTLRegionMake2D(0, 0, kIsomapsWatermarkW, kIsomapsWatermarkH)
                mipmapLevel:0
                  withBytes:kIsomapsWatermarkRGBA
                bytesPerRow:kIsomapsWatermarkW * 4];

  MTLSamplerDescriptor* sd = [[MTLSamplerDescriptor alloc] init];
  sd.minFilter = MTLSamplerMinMagFilterLinear;
  sd.magFilter = MTLSamplerMinMagFilterLinear;
  sd.sAddressMode = MTLSamplerAddressModeClampToEdge;
  sd.tAddressMode = MTLSamplerAddressModeClampToEdge;
  sWmSampler = [device newSamplerStateWithDescriptor:sd];
}

+ (void)drawInto:(id<MTLCommandBuffer>)cmd
        drawable:(id<CAMetalDrawable>)drawable
          device:(id<MTLDevice>)device {
  if (!cmd || !drawable || !device) return;
  id<MTLTexture> target = drawable.texture;
  if (!target) return;
  [self ensure:device pixelFormat:target.pixelFormat];
  if (!sWmPipeline || !sWmTexture || !sWmSampler) return;

  const float fbW = (float)target.width, fbH = (float)target.height;
  if (fbW < 2 || fbH < 2) return;
  const float wpx = std::min(std::max(fbW * 0.22f, 90.0f), 340.0f);
  const float hpx = wpx * (float)kIsomapsWatermarkH / (float)kIsomapsWatermarkW;
  const float m = fbW * 0.03f;
  simd::float4 rect = {(fbW - wpx - m) / fbW, m / fbH, wpx / fbW, hpx / fbH};  // x,y,w,h in [0,1], y up

  MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
  rp.colorAttachments[0].texture = target;
  rp.colorAttachments[0].loadAction = MTLLoadActionLoad;   // conserve la carte déjà rendue
  rp.colorAttachments[0].storeAction = MTLStoreActionStore;
  id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:rp];
  [enc setRenderPipelineState:sWmPipeline];
  [enc setVertexBytes:&rect length:sizeof(rect) atIndex:0];
  [enc setFragmentTexture:sWmTexture atIndex:0];
  [enc setFragmentSamplerState:sWmSampler atIndex:0];
  [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4];
  [enc endEncoding];
}

@end
#endif  // ISOMAPS_SDK_WATERMARK

@interface MLNMapViewImplDelegate : NSObject <MTKViewDelegate>
@end

@implementation MLNMapViewImplDelegate {
  MLNMapViewMetalImpl* _impl;
}

- (instancetype)initWithImpl:(MLNMapViewMetalImpl*)impl {
  if (self = [super init]) {
    _impl = impl;
  }
  return self;
}

- (void)mtkView:(MTKView*)view drawableSizeWillChange:(CGSize)size {
}

- (void)drawInMTKView:(MTKView*)view {
  _impl->render();
}

@end

class MLNMapViewMetalRenderableResource final : public mbgl::mtl::RenderableResource {
public:
  MLNMapViewMetalRenderableResource(MLNMapViewMetalImpl& backend_)
      : backend(backend_), delegate([[MLNMapViewImplDelegate alloc] initWithImpl:&backend]) {}

  void bind() override {
    if (!commandQueue) {
      commandQueue = [mtlView.device newCommandQueue];
    }

    if (!commandBuffer) {
      commandBuffer = [commandQueue commandBuffer];
      commandBufferPtr = NS::RetainPtr((__bridge MTL::CommandBuffer*)commandBuffer);
    }
  }

  const mbgl::mtl::RendererBackend& getBackend() const override { return backend; }

  const mbgl::mtl::MTLCommandBufferPtr& getCommandBuffer() const override {
    return commandBufferPtr;
  }

  virtual mbgl::mtl::MTLBlitPassDescriptorPtr getUploadPassDescriptor() const override {
    // Create from render pass descriptor?
    return NS::TransferPtr(MTL::BlitPassDescriptor::alloc()->init());
  }

  const mbgl::mtl::MTLRenderPassDescriptorPtr& getRenderPassDescriptor() const override {
    if (!cachedRenderPassDescriptor) {
      auto* mtlDesc = mtlView.currentRenderPassDescriptor;
      cachedRenderPassDescriptor = NS::RetainPtr((__bridge MTL::RenderPassDescriptor*)mtlDesc);
    }
    return cachedRenderPassDescriptor;
  }

  void swap() override {
    id<CAMetalDrawable> currentDrawable = [mtlView currentDrawable];
    if (currentDrawable) {
#if defined(ISOMAPS_SDK_WATERMARK)
      // Logo Isomaps incrusté (build SDK vendu uniquement) — dessiné sur le drawable
      // avant présentation, sur le command buffer courant.
      [MLNIsomapsWatermark drawInto:commandBuffer
                           drawable:currentDrawable
                             device:mtlView.device];
#endif
      if (presentsWithTransaction) {
        [commandBuffer commit];
        [commandBuffer waitUntilCompleted];
        [currentDrawable present];
      } else {
        [commandBuffer presentDrawable:currentDrawable];
        [commandBuffer commit];
      }
    }

    commandBuffer = nil;
    commandBufferPtr.reset();

    cachedRenderPassDescriptor.reset();
  }

  mbgl::Size framebufferSize() {
    assert(mtlView);
    return {static_cast<uint32_t>(mtlView.drawableSize.width),
            static_cast<uint32_t>(mtlView.drawableSize.height)};
  }

private:
  MLNMapViewMetalImpl& backend;
  mbgl::mtl::MTLCommandBufferPtr commandBufferPtr;
  mutable mbgl::mtl::MTLRenderPassDescriptorPtr cachedRenderPassDescriptor;

public:
  MLNMapViewImplDelegate* delegate = nil;
  MTKView* mtlView = nil;
  id<MTLCommandBuffer> commandBuffer;
  id<MTLCommandQueue> commandQueue;
  bool presentsWithTransaction = false;

  // Cached last-applied values comparing against MTKView's reflected state round-trips
  // through UIKit and can defeat the no-op guard under non-integer scale factors.
  CGFloat lastAppliedScaleFactor = 0;
  CGSize lastAppliedDrawableSize = CGSizeZero;

  // We count how often the context was activated/deactivated so that we can truly deactivate it
  // after the activation count drops to 0.
  NSUInteger activationCount = 0;
};

MLNMapViewMetalImpl::MLNMapViewMetalImpl(MLNMapView* nativeView_)
    : MLNMapViewImpl(nativeView_),
      mbgl::mtl::RendererBackend(mbgl::gfx::ContextMode::Unique),
      mbgl::gfx::Renderable({0, 0}, std::make_unique<MLNMapViewMetalRenderableResource>(*this)) {}

MLNMapViewMetalImpl::~MLNMapViewMetalImpl() = default;

void MLNMapViewMetalImpl::setOpaque(const bool opaque) {
  auto& resource = getResource<MLNMapViewMetalRenderableResource>();
  resource.mtlView.opaque = opaque;
  resource.mtlView.layer.opaque = opaque;
}

void MLNMapViewMetalImpl::setPresentsWithTransaction(const bool value) {
  auto& resource = getResource<MLNMapViewMetalRenderableResource>();
  resource.presentsWithTransaction = value;

  if (@available(iOS 13.0, *)) {
    if (CAMetalLayer* metalLayer = MLN_OBJC_DYNAMIC_CAST(resource.mtlView.layer, CAMetalLayer)) {
      metalLayer.presentsWithTransaction = value;
    }
  }
}

void MLNMapViewMetalImpl::display() {
  auto& resource = getResource<MLNMapViewMetalRenderableResource>();

  // Calling `display` here directly causes the stuttering bug (if
  // `presentsWithTransaction` is `YES` - see above)
  // as reported in https://github.com/mapbox/mapbox-gl-native-ios/issues/350
  //
  // Since we use `presentsWithTransaction` to synchronize with UIView
  // annotations, we now let the system handle when the view is rendered. This
  // has the potential to increase latency
  [resource.mtlView setNeedsDisplay];
}

void MLNMapViewMetalImpl::createView() {
  auto& resource = getResource<MLNMapViewMetalRenderableResource>();
  if (resource.mtlView) {
    return;
  }

  id<MTLDevice> device = (__bridge id<MTLDevice>)resource.getBackend().getDevice().get();
  const auto scaleFactor = MLNEffectiveScaleFactorForView(mapView);

  resource.mtlView = [[MTKView alloc] initWithFrame:mapView.bounds device:device];
  resource.mtlView.delegate = resource.delegate;
  resource.mtlView.autoresizingMask =
      UIViewAutoresizingFlexibleWidth | UIViewAutoresizingFlexibleHeight;
  // We own drawable sizing in layoutChanged(); disable MTKView's automatic recompute so
  // setting contentScaleFactor cannot race with explicit drawableSize assignment.
  resource.mtlView.autoResizeDrawable = NO;
  resource.mtlView.contentScaleFactor = scaleFactor;
  resource.mtlView.contentMode = UIViewContentModeCenter;
  resource.mtlView.colorPixelFormat = MTLPixelFormatBGRA8Unorm;
  resource.mtlView.depthStencilPixelFormat = MTLPixelFormatDepth32Float_Stencil8;
  resource.mtlView.opaque = mapView.opaque;
  resource.mtlView.layer.opaque = mapView.opaque;
  resource.mtlView.enableSetNeedsDisplay = YES;
  if (@available(iOS 13.0, *)) {
    CAMetalLayer* metalLayer = MLN_OBJC_DYNAMIC_CAST(resource.mtlView.layer, CAMetalLayer);
    metalLayer.presentsWithTransaction = resource.presentsWithTransaction;
  }

  [mapView insertSubview:resource.mtlView atIndex:0];
}

UIView* MLNMapViewMetalImpl::getView() {
  auto& resource = getResource<MLNMapViewMetalRenderableResource>();
  return resource.mtlView;
}

void MLNMapViewMetalImpl::deleteView() {
  auto& resource = getResource<MLNMapViewMetalRenderableResource>();
  [resource.mtlView releaseDrawables];
}

void MLNMapViewMetalImpl::activate() {
  auto& resource = getResource<MLNMapViewMetalRenderableResource>();
  if (resource.activationCount++) {
    return;
  }
}

void MLNMapViewMetalImpl::deactivate() {
  auto& resource = getResource<MLNMapViewMetalRenderableResource>();
  if (--resource.activationCount) {
    return;
  }
}

/// This function is called before we start rendering, when iOS invokes our rendering method.
/// iOS already sets the correct framebuffer and viewport for us, so we need to update the
/// context state with the anticipated values.
void MLNMapViewMetalImpl::updateAssumedState() {
  auto& resource = getResource<MLNMapViewMetalRenderableResource>();
  assumeFramebufferBinding(ImplicitFramebufferBinding);
  assumeViewport(0, 0, resource.framebufferSize());
}

UIImage* MLNMapViewMetalImpl::snapshot() {
  auto& resource = getResource<MLNMapViewMetalRenderableResource>();
  return nil;  // TODO: resource.mtlView.snapshot;
}

void MLNMapViewMetalImpl::layoutChanged() {
  // Fix: unconditionally rewriting contentScaleFactor/drawableSize every
  // layout pass caused a feedback loop under iOS 26 Smart Display Zoom (non-integer scale).
  // Also skip pre-layout/detached passes — MLNEffectiveScaleFactorForView falls back to
  // mainScreen without a window, wrong for CarPlay.
  auto& resource = getResource<MLNMapViewMetalRenderableResource>();
  const auto viewSize = mapView.bounds.size;
  if (!resource.mtlView || viewSize.width <= 0 || viewSize.height <= 0 || !mapView.window) {
    return;
  }

  const auto scaleFactor = MLNEffectiveScaleFactorForView(mapView);
  const CGSize target = CGSizeMake(std::round(viewSize.width * scaleFactor),
                                   std::round(viewSize.height * scaleFactor));

  if (scaleFactor != resource.lastAppliedScaleFactor) {
    resource.mtlView.contentScaleFactor = resource.lastAppliedScaleFactor = scaleFactor;
  }
  if (!CGSizeEqualToSize(target, resource.lastAppliedDrawableSize)) {
    resource.mtlView.drawableSize = resource.lastAppliedDrawableSize = target;
  }

  size = {static_cast<uint32_t>(target.width), static_cast<uint32_t>(target.height)};
}

MLNBackendResource* MLNMapViewMetalImpl::getObject() {
  auto& resource = getResource<MLNMapViewMetalRenderableResource>();
  auto renderPassDescriptor = resource.getRenderPassDescriptor().get();

  return [[MLNBackendResource alloc] initWithMTKView:resource.mtlView
                                              device:resource.mtlView.device
                                renderPassDescriptor:[MTLRenderPassDescriptor renderPassDescriptor]
                                       commandBuffer:resource.commandBuffer];
}
